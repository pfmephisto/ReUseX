// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/cases.hpp"

#include "api/api.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <ctime>
#include <fstream>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <system_error>
#include <utility>

namespace ruxd::api {
namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

constexpr std::size_t kMaxSlugLength = 64;
constexpr std::size_t kMaxNameLength = 200;
constexpr std::string_view kCaseFileName = "project.rux";
constexpr std::string_view kMetaDir = ".ruxd";

/// ASCII folding for the Latin letters a Danish (or Nordic/German) project
/// name is likely to hold, as UTF-8 byte sequences.
constexpr std::array<std::pair<std::string_view, std::string_view>, 20> kFolds{{
    {"\xc3\xa6", "ae"}, {"\xc3\x86", "ae"}, // æ Æ
    {"\xc3\xb8", "oe"}, {"\xc3\x98", "oe"}, // ø Ø
    {"\xc3\xa5", "aa"}, {"\xc3\x85", "aa"}, // å Å
    {"\xc3\xa4", "ae"}, {"\xc3\x84", "ae"}, // ä Ä
    {"\xc3\xb6", "oe"}, {"\xc3\x96", "oe"}, // ö Ö
    {"\xc3\xbc", "ue"}, {"\xc3\x9c", "ue"}, // ü Ü
    {"\xc3\x9f", "ss"},                     // ß
    {"\xc3\xa9", "e"},  {"\xc3\x89", "e"},  // é É
    {"\xc3\xa8", "e"},  {"\xc3\xa1", "a"},  // è á
    {"\xc3\xad", "i"},  {"\xc3\xb3", "o"},  // í ó
    {"\xc3\xba", "u"},                      // ú
}};

std::string iso8601(std::chrono::system_clock::time_point tp) {
  const std::time_t seconds = std::chrono::system_clock::to_time_t(tp);
  std::tm utc{};
  gmtime_r(&seconds, &utc);
  return fmt::format("{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z",
                     utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                     utc.tm_hour, utc.tm_min, utc.tm_sec);
}

std::string mtime_iso(const fs::path &file) {
  std::error_code ec;
  const auto ftime = fs::last_write_time(file, ec);
  if (ec)
    return {};
  return iso8601(std::chrono::file_clock::to_sys(ftime));
}

std::string trim(std::string_view text) {
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string_view::npos)
    return {};
  const auto last = text.find_last_not_of(" \t\r\n");
  return std::string(text.substr(first, last - first + 1));
}

/// A case name: trimmed, 1–200 bytes, no control characters.
std::string validate_name(std::string_view raw) {
  std::string name = trim(raw);
  if (name.empty())
    throw HttpError(400, "'name' must not be empty");
  if (name.size() > kMaxNameLength)
    throw HttpError(400, "'name' is longer than " +
                             std::to_string(kMaxNameLength) + " bytes");
  if (std::any_of(name.begin(), name.end(),
                  [](unsigned char c) { return c < 0x20 || c == 0x7f; }))
    throw HttpError(400, "'name' must not contain control characters");
  return name;
}

/// 128 random bits as 32 lower-case hex digits (the OS's entropy source).
std::string random_hex_id() {
  std::random_device rd;
  std::string out;
  out.reserve(32);
  for (int i = 0; i < 4; ++i)
    out += fmt::format("{:08x}", static_cast<std::uint32_t>(rd()));
  return out;
}

bool is_upload_id(std::string_view id) {
  return id.size() == 32 &&
         std::all_of(id.begin(), id.end(), [](unsigned char c) {
           return std::isdigit(c) != 0 || (c >= 'a' && c <= 'f');
         });
}

std::uintmax_t file_size_or_zero(const fs::path &file) {
  std::error_code ec;
  const auto size = fs::file_size(file, ec);
  return ec ? 0 : size;
}

/// True when @p child is @p parent or inside it (both canonical-ish).
bool is_within(const fs::path &child, const fs::path &parent) {
  if (parent.empty())
    return false;
  const auto c = fs::weakly_canonical(child);
  const auto p = fs::weakly_canonical(parent);
  auto ci = c.begin();
  for (auto pi = p.begin(); pi != p.end(); ++pi, ++ci) {
    if (pi->empty())
      continue; // trailing separator
    if (ci == c.end() || *ci != *pi)
      return false;
  }
  return true;
}

void write_file_atomically(const fs::path &target, const std::string &text) {
  const fs::path tmp = target.string() + ".tmp";
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out)
      throw std::runtime_error("cannot write " + tmp.string());
    out << text;
    if (!out)
      throw std::runtime_error("cannot write " + tmp.string());
  }
  fs::rename(tmp, target);
}

} // namespace

// ===========================================================================
// Ids and bodies
// ===========================================================================

std::string case_slug(std::string_view name) {
  std::string folded;
  folded.reserve(name.size());
  for (std::size_t i = 0; i < name.size();) {
    bool matched = false;
    for (const auto &[from, to] : kFolds) {
      if (name.substr(i, from.size()) == from) {
        folded += to;
        i += from.size();
        matched = true;
        break;
      }
    }
    if (!matched)
      folded += name[i++];
  }

  std::string slug;
  bool dash = false;
  for (unsigned char c : folded) {
    if (std::isalnum(c) != 0 && c < 0x80) {
      if (dash && !slug.empty())
        slug += '-';
      dash = false;
      slug += static_cast<char>(std::tolower(c));
    } else {
      dash = true;
    }
  }
  if (slug.size() > kMaxSlugLength) {
    slug.resize(kMaxSlugLength);
    while (!slug.empty() && slug.back() == '-')
      slug.pop_back();
  }
  return slug.empty() ? std::string("sag") : slug;
}

std::vector<std::string>
assign_case_ids(const std::vector<std::string> &names) {
  std::vector<std::string> ids;
  ids.reserve(names.size());
  std::set<std::string> taken;
  for (const auto &name : names) {
    const std::string base = case_slug(name);
    std::string id = base;
    for (int n = 2; taken.count(id) != 0; ++n) {
      const std::string suffix = "-" + std::to_string(n);
      id = base.substr(0, kMaxSlugLength - suffix.size()) + suffix;
    }
    taken.insert(id);
    ids.push_back(std::move(id));
  }
  return ids;
}

CasePatch parse_case_patch(std::string_view body) {
  auto parsed = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object())
    throw HttpError(400, "request body must be a JSON object");
  CasePatch patch;
  for (const auto &[key, value] : parsed.items()) {
    if (key == "name") {
      if (!value.is_string())
        throw HttpError(400, "'name' must be a string");
      patch.name = validate_name(value.get<std::string>());
    } else if (key == "archived") {
      if (!value.is_boolean())
        throw HttpError(400, "'archived' must be a boolean");
      patch.archived = value.get<bool>();
    } else {
      throw HttpError(400, "unknown field '" + key + "'");
    }
  }
  return patch;
}

std::string parse_case_create(std::string_view body) {
  auto parsed = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object())
    throw HttpError(400, "request body must be a JSON object");
  auto it = parsed.find("name");
  if (it == parsed.end() || !it->is_string())
    throw HttpError(400, "'name' is required and must be a string");
  return validate_name(it->get<std::string>());
}

bool has_sqlite_header(const fs::path &file) {
  static constexpr std::string_view kMagic{"SQLite format 3\0", 16};
  std::ifstream in(file, std::ios::binary);
  std::array<char, 16> head{};
  if (!in.read(head.data(), head.size()))
    return false;
  return std::string_view(head.data(), head.size()) == kMagic;
}

// ===========================================================================
// LocalCaseStore
// ===========================================================================

class LocalCaseStore::Impl {
    public:
  Impl(fs::path target, fs::path data_dir) {
    if (target.empty())
      throw std::runtime_error("no project file or directory given");
    std::error_code ec;
    if (fs::is_directory(target, ec)) {
      target_dir_ = fs::absolute(target);
      if (data_dir.empty())
        data_dir = target_dir_;
    } else {
      if (target.extension() != ".rux")
        throw std::runtime_error("'" + target.string() +
                                 "' is neither a directory nor a .rux file");
      target_file_ = fs::absolute(target);
    }
    if (!data_dir.empty()) {
      fs::create_directories(data_dir);
      data_dir_ = fs::canonical(data_dir);
      fs::create_directories(meta_dir() / "trash");
      fs::create_directories(meta_dir() / "uploads");
      load_meta();
    }
  }

  // --- scanning -------------------------------------------------------------

  struct Found {
    fs::path file;
    std::string base; ///< Stem or directory name, for the slug and name.
    std::string key;  ///< Metadata key.
    bool in_data_dir = false;
  };

  std::vector<CaseInfo> scan() const {
    std::vector<Found> found;
    std::set<fs::path> seen;
    auto add = [&](Found f) {
      const auto canon = fs::weakly_canonical(f.file);
      if (!seen.insert(canon).second)
        return;
      f.key = meta_key(f.file);
      f.in_data_dir = is_within(f.file, data_dir_);
      found.push_back(std::move(f));
    };

    if (!target_file_.empty())
      add({target_file_, target_file_.stem().string(), {}, false});
    if (!target_dir_.empty())
      for (auto &f : flat_files(target_dir_))
        add(std::move(f));
    if (!data_dir_.empty()) {
      if (data_dir_ != fs::weakly_canonical(target_dir_))
        for (auto &f : flat_files(data_dir_))
          add(std::move(f));
      for (auto &f : case_dirs(data_dir_))
        add(std::move(f));
    }

    // Stable order for id assignment: by path relative to its root.
    std::sort(found.begin(), found.end(), [](const Found &a, const Found &b) {
      return a.file.generic_string() < b.file.generic_string();
    });
    std::vector<std::string> bases;
    bases.reserve(found.size());
    for (const auto &f : found)
      bases.push_back(f.base);
    const auto ids = assign_case_ids(bases);

    std::vector<CaseInfo> cases;
    cases.reserve(found.size());
    std::lock_guard<std::mutex> lock(meta_mutex_);
    for (std::size_t i = 0; i < found.size(); ++i) {
      const Found &f = found[i];
      CaseInfo info;
      info.id = ids[i];
      info.path = f.file;
      info.name = f.base;
      info.size_bytes = file_size_or_zero(f.file);
      info.deletable = f.in_data_dir;
      auto meta = meta_.find(f.key);
      if (meta != meta_.end()) {
        if (auto n = meta->second.find("name");
            n != meta->second.end() && n->is_string())
          info.name = n->get<std::string>();
        if (auto a = meta->second.find("archived");
            a != meta->second.end() && a->is_boolean())
          info.archived = a->get<bool>();
        if (auto c = meta->second.find("created_at");
            c != meta->second.end() && c->is_string())
          info.created_at = c->get<std::string>();
      }
      if (info.created_at.empty())
        info.created_at = mtime_iso(f.file);
      cases.push_back(std::move(info));
    }
    std::sort(cases.begin(), cases.end(),
              [](const CaseInfo &a, const CaseInfo &b) { return a.id < b.id; });
    return cases;
  }

  std::optional<CaseInfo> find(std::string_view id) const {
    for (auto &info : scan())
      if (info.id == id)
        return std::move(info);
    return std::nullopt;
  }

  bool writable() const { return !data_dir_.empty(); }

  fs::path staging_dir() const {
    return writable() ? meta_dir() / "uploads" : fs::path{};
  }

  // --- mutation -------------------------------------------------------------

  CaseInfo create(const std::string &name) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    const fs::path dir = new_case_dir(name);
    return finish_create(dir, name);
  }

  CaseInfo adopt(const std::string &name, const fs::path &staged) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    const fs::path dir = new_case_dir(name);
    std::error_code ec;
    fs::rename(staged, dir / kCaseFileName, ec);
    if (ec) {
      fs::remove(dir);
      throw std::runtime_error("could not move the upload into place: " +
                               ec.message());
    }
    return finish_create(dir, name);
  }

  CaseInfo update(std::string_view id, const CasePatch &patch) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    auto info = find(id);
    if (!info)
      throw HttpError(404, "no such case '" + std::string(id) + "'");
    {
      std::lock_guard<std::mutex> meta_lock(meta_mutex_);
      json &entry = meta_[meta_key(info->path)];
      if (!entry.is_object())
        entry = json::object();
      if (patch.name)
        entry["name"] = *patch.name;
      if (patch.archived)
        entry["archived"] = *patch.archived;
      save_meta_locked();
    }
    return *find(id);
  }

  fs::path move_to_trash(std::string_view id) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    auto info = find(id);
    if (!info)
      throw HttpError(404, "no such case '" + std::string(id) + "'");
    if (!info->deletable)
      throw HttpError(409, "case '" + info->id +
                               "' is not in the server's data dir and cannot "
                               "be deleted here");

    const auto stamp = iso8601(std::chrono::system_clock::now());
    std::string safe_stamp;
    for (char c : stamp)
      safe_stamp += (c == ':' ? '-' : c);
    fs::path dest = meta_dir() / "trash" / (safe_stamp + "-" + info->id);
    for (int n = 2; fs::exists(dest); ++n)
      dest = meta_dir() / "trash" /
             (safe_stamp + "-" + info->id + "-" + std::to_string(n));

    const bool own_dir = info->path.filename() == kCaseFileName &&
                         info->path.parent_path().parent_path() == data_dir_;
    if (own_dir) {
      fs::rename(info->path.parent_path(), dest);
    } else {
      fs::create_directory(dest);
      for (const std::string suffix : {"", "-wal", "-shm"}) {
        const fs::path from = info->path.string() + suffix;
        std::error_code ec;
        if (fs::exists(from, ec))
          fs::rename(from, dest / (info->path.filename().string() + suffix));
      }
    }
    {
      std::lock_guard<std::mutex> meta_lock(meta_mutex_);
      meta_.erase(meta_key(info->path));
      save_meta_locked();
    }
    spdlog::info("Case '{}' moved to {}", info->id, dest.string());
    return dest;
  }

  const fs::path &data_dir() const noexcept { return data_dir_; }

    private:
  fs::path meta_dir() const { return data_dir_ / kMetaDir; }

  static bool hidden(const fs::path &p) {
    const auto name = p.filename().string();
    return !name.empty() && name.front() == '.';
  }

  /// `*.rux` regular files (or links to them) directly inside @p dir.
  static std::vector<Found> flat_files(const fs::path &dir) {
    std::vector<Found> out;
    std::error_code ec;
    for (const auto &entry : fs::directory_iterator(dir, ec)) {
      const auto &p = entry.path();
      if (hidden(p) || p.extension() != ".rux")
        continue;
      std::error_code type_ec;
      if (!entry.is_regular_file(type_ec))
        continue;
      out.push_back({p, p.stem().string(), {}, false});
    }
    return out;
  }

  /// `<dir>/<name>/project.rux` for every real (non-symlink) sub-directory.
  static std::vector<Found> case_dirs(const fs::path &dir) {
    std::vector<Found> out;
    std::error_code ec;
    for (const auto &entry : fs::directory_iterator(dir, ec)) {
      const auto &p = entry.path();
      std::error_code type_ec;
      if (hidden(p) || entry.is_symlink(type_ec) ||
          !entry.is_directory(type_ec))
        continue;
      const fs::path file = p / kCaseFileName;
      if (!fs::is_regular_file(file, type_ec) || fs::is_symlink(file, type_ec))
        continue;
      out.push_back({file, p.filename().string(), {}, false});
    }
    return out;
  }

  std::string meta_key(const fs::path &file) const {
    if (!data_dir_.empty() && is_within(file, data_dir_))
      return fs::relative(fs::weakly_canonical(file), data_dir_)
          .generic_string();
    return fs::weakly_canonical(file).generic_string();
  }

  /// A fresh, empty case directory named by a slug no case uses yet.
  fs::path new_case_dir(const std::string &name) {
    if (!writable())
      throw HttpError(409, "this server has no data dir: it serves one file "
                           "and cannot create or upload cases (start it with "
                           "a directory or --data-dir)");
    std::set<std::string> taken;
    for (const auto &info : scan())
      taken.insert(info.id);
    const std::string base = case_slug(name);
    for (int n = 1; n < 10000; ++n) {
      const std::string suffix = n == 1 ? "" : "-" + std::to_string(n);
      const std::string slug =
          base.substr(0, kMaxSlugLength - suffix.size()) + suffix;
      if (taken.count(slug) != 0)
        continue;
      const fs::path dir = data_dir_ / slug;
      std::error_code ec;
      // create_directory (not create_directories) fails on an existing path,
      // so a race or a stray entry can never be reused.
      if (fs::create_directory(dir, ec) && !ec)
        return dir;
    }
    throw HttpError(409,
                    "could not find a free directory name for '" + name + "'");
  }

  CaseInfo finish_create(const fs::path &dir, const std::string &name) {
    const fs::path file = dir / kCaseFileName;
    try {
      // Creates and migrates a new project; validates (and migrates) an
      // adopted upload. Either way the case is openable once listed.
      reusex::ProjectDB db(file, /*readOnly=*/false);
    } catch (const std::exception &e) {
      std::error_code ec;
      fs::remove_all(dir, ec);
      throw HttpError(422,
                      std::string("not a usable ReUseX project: ") + e.what());
    }
    {
      std::lock_guard<std::mutex> meta_lock(meta_mutex_);
      meta_[meta_key(file)] =
          json{{"name", name},
               {"created_at", iso8601(std::chrono::system_clock::now())}};
      save_meta_locked();
    }
    const auto canon = fs::weakly_canonical(file);
    for (auto &info : scan())
      if (fs::weakly_canonical(info.path) == canon)
        return info;
    throw std::runtime_error("the new case at " + file.string() +
                             " is not listed");
  }

  void load_meta() {
    const fs::path file = meta_dir() / "cases.json";
    std::error_code ec;
    if (!fs::exists(file, ec))
      return;
    std::ifstream in(file);
    auto parsed = json::parse(in, nullptr, /*allow_exceptions=*/false);
    if (parsed.is_discarded() || !parsed.is_object()) {
      spdlog::warn("Ignoring unreadable case metadata {}", file.string());
      return;
    }
    for (auto &[key, value] : parsed.items())
      if (value.is_object())
        meta_[key] = value;
  }

  /// Caller holds meta_mutex_. No data dir: metadata stays in memory.
  void save_meta_locked() {
    if (data_dir_.empty())
      return;
    json out = json::object();
    for (const auto &[key, value] : meta_)
      out[key] = value;
    write_file_atomically(meta_dir() / "cases.json", out.dump(2));
  }

  fs::path target_file_;
  fs::path target_dir_;
  fs::path data_dir_;

  std::mutex write_mutex_; ///< Serializes create/adopt/update/delete.
  mutable std::mutex meta_mutex_;
  std::map<std::string, json> meta_;
};

LocalCaseStore::LocalCaseStore(fs::path target, fs::path data_dir)
    : impl_(std::make_unique<Impl>(std::move(target), std::move(data_dir))) {}
LocalCaseStore::~LocalCaseStore() = default;
std::vector<CaseInfo> LocalCaseStore::list() const { return impl_->scan(); }
std::optional<CaseInfo> LocalCaseStore::find(std::string_view id) const {
  return impl_->find(id);
}
bool LocalCaseStore::writable() const { return impl_->writable(); }
fs::path LocalCaseStore::staging_dir() const { return impl_->staging_dir(); }
CaseInfo LocalCaseStore::create(const std::string &name) {
  return impl_->create(name);
}
CaseInfo LocalCaseStore::adopt(const std::string &name, const fs::path &file) {
  return impl_->adopt(name, file);
}
CaseInfo LocalCaseStore::update(std::string_view id, const CasePatch &patch) {
  return impl_->update(id, patch);
}
fs::path LocalCaseStore::move_to_trash(std::string_view id) {
  return impl_->move_to_trash(id);
}
const fs::path &LocalCaseStore::data_dir() const noexcept {
  return impl_->data_dir();
}

// ===========================================================================
// UploadManager
// ===========================================================================

class UploadManager::Impl {
    public:
  Impl(fs::path dir, UploadLimits limits)
      : dir_(std::move(dir)), limits_(limits) {
    if (dir_.empty())
      return;
    fs::create_directories(dir_);
    // Nothing survives a restart: an upload's session lives in memory, so a
    // leftover .part file can never be resumed.
    std::error_code ec;
    for (const auto &entry : fs::directory_iterator(dir_, ec))
      if (entry.path().extension() == ".part")
        fs::remove(entry.path(), ec);
  }

  UploadSession begin(const std::string &raw_name, std::uint64_t size) {
    if (dir_.empty())
      throw HttpError(409, "this server cannot accept uploads (no data dir)");
    const std::string name = validate_name(raw_name);
    if (size == 0)
      throw HttpError(400, "'size' must be a positive number of bytes");
    if (size > limits_.max_bytes)
      throw HttpError(413, fmt::format("file is {} bytes; this server accepts "
                                       "at most {}",
                                       size, limits_.max_bytes));
    expire(Clock::now());
    std::lock_guard<std::mutex> lock(mutex_);
    if (sessions_.size() >= limits_.max_sessions)
      throw HttpError(429, "too many uploads in progress; try again later");
    Entry entry;
    entry.session.id = random_hex_id();
    entry.session.name = name;
    entry.session.size = size;
    entry.touched = Clock::now();
    std::ofstream create(part_path(entry.session.id), std::ios::binary);
    if (!create)
      throw HttpError(500, "could not create the staging file");
    auto session = entry.session;
    sessions_.emplace(session.id, std::move(entry));
    return session;
  }

  UploadSession append(std::string_view id, std::uint64_t offset,
                       std::string_view bytes) {
    if (bytes.size() > limits_.max_chunk_bytes)
      throw HttpError(413, fmt::format("chunk is {} bytes; at most {} per "
                                       "request",
                                       bytes.size(), limits_.max_chunk_bytes));
    std::lock_guard<std::mutex> lock(mutex_);
    Entry &entry = get(id);
    if (offset != entry.session.received)
      throw HttpError(409, fmt::format("offset {} does not continue the "
                                       "upload; expected offset {}",
                                       offset, entry.session.received));
    if (entry.session.received + bytes.size() > entry.session.size)
      throw HttpError(413, fmt::format("chunk would grow the upload past its "
                                       "declared {} bytes",
                                       entry.session.size));
    std::ofstream out(part_path(entry.session.id),
                      std::ios::binary | std::ios::app);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    out.flush();
    if (!out)
      throw HttpError(507, "could not write the upload to disk");
    entry.session.received += bytes.size();
    entry.touched = Clock::now();
    return entry.session;
  }

  UploadSession status(std::string_view id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return const_cast<Impl *>(this)->get(id).session;
  }

  std::pair<UploadSession, fs::path> finish(std::string_view id) {
    std::lock_guard<std::mutex> lock(mutex_);
    Entry &entry = get(id);
    if (entry.session.received != entry.session.size)
      throw HttpError(409,
                      fmt::format("upload incomplete: {} of {} bytes "
                                  "received",
                                  entry.session.received, entry.session.size));
    auto out = std::make_pair(entry.session, part_path(entry.session.id));
    sessions_.erase(std::string(id));
    return out;
  }

  void abort(std::string_view id) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_upload_id(id))
      return;
    auto it = sessions_.find(std::string(id));
    if (it == sessions_.end())
      return;
    std::error_code ec;
    fs::remove(part_path(it->first), ec);
    sessions_.erase(it);
  }

  std::size_t expire(Clock::time_point now) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t dropped = 0;
    for (auto it = sessions_.begin(); it != sessions_.end();) {
      if (now - it->second.touched > limits_.idle_ttl) {
        std::error_code ec;
        fs::remove(part_path(it->first), ec);
        it = sessions_.erase(it);
        ++dropped;
      } else {
        ++it;
      }
    }
    return dropped;
  }

  const UploadLimits &limits() const noexcept { return limits_; }

    private:
  struct Entry {
    UploadSession session;
    Clock::time_point touched;
  };

  /// Caller holds mutex_. The id shape is checked before it names a file.
  Entry &get(std::string_view id) {
    if (!is_upload_id(id))
      throw HttpError(404, "no such upload");
    auto it = sessions_.find(std::string(id));
    if (it == sessions_.end())
      throw HttpError(404, "no such upload");
    return it->second;
  }

  fs::path part_path(const std::string &id) const {
    return dir_ / (id + ".part");
  }

  fs::path dir_;
  UploadLimits limits_;
  mutable std::mutex mutex_;
  std::map<std::string, Entry> sessions_;
};

UploadManager::UploadManager(fs::path staging_dir, UploadLimits limits)
    : impl_(std::make_unique<Impl>(std::move(staging_dir), limits)) {}
UploadManager::~UploadManager() = default;
const UploadLimits &UploadManager::limits() const noexcept {
  return impl_->limits();
}
UploadSession UploadManager::begin(const std::string &name,
                                   std::uint64_t size) {
  return impl_->begin(name, size);
}
UploadSession UploadManager::append(std::string_view id, std::uint64_t offset,
                                    std::string_view bytes) {
  return impl_->append(id, offset, bytes);
}
UploadSession UploadManager::status(std::string_view id) const {
  return impl_->status(id);
}
std::pair<UploadSession, fs::path> UploadManager::finish(std::string_view id) {
  return impl_->finish(id);
}
void UploadManager::abort(std::string_view id) { impl_->abort(id); }
std::size_t UploadManager::expire(Clock::time_point now) {
  return impl_->expire(now);
}

// ===========================================================================
// Wire shapes
// ===========================================================================

json case_json(const CaseInfo &info, bool open) {
  return json{{"id", info.id},
              {"name", info.name},
              {"file_name", info.path.filename().string()},
              {"created_at", info.created_at},
              {"archived", info.archived},
              {"size_bytes", info.size_bytes},
              {"deletable", info.deletable},
              {"open", open}};
}

json cases_list_json(json cases, bool writable, const UploadLimits &limits) {
  return json{{"cases", std::move(cases)},
              {"writable", writable},
              {"upload",
               {{"max_bytes", limits.max_bytes},
                {"chunk_bytes", limits.max_chunk_bytes}}}};
}

UploadRequest parse_upload_request(std::string_view body) {
  auto parsed = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object())
    throw HttpError(400, "request body must be a JSON object");
  UploadRequest out;
  auto name = parsed.find("name");
  if (name == parsed.end() || !name->is_string())
    throw HttpError(400, "'name' is required and must be a string");
  out.name = name->get<std::string>();
  auto size = parsed.find("size");
  if (size == parsed.end() || !size->is_number_integer() ||
      size->get<std::int64_t>() <= 0)
    throw HttpError(400, "'size' is required and must be a positive integer");
  out.size = size->get<std::uint64_t>();
  return out;
}

std::uint64_t parse_upload_offset(const char *raw) {
  if (raw == nullptr || *raw == '\0')
    throw HttpError(400, "'offset' is required");
  const std::string_view text(raw);
  if (text.size() > 19 ||
      !std::all_of(text.begin(), text.end(),
                   [](unsigned char c) { return std::isdigit(c) != 0; }))
    throw HttpError(400, "'offset' must be a non-negative integer");
  return std::stoull(std::string(text));
}

json upload_json(const UploadSession &session, const UploadLimits &limits) {
  return json{{"id", session.id},
              {"name", session.name},
              {"size", session.size},
              {"received", session.received},
              {"chunk_bytes", limits.max_chunk_bytes}};
}

std::optional<std::string> case_id_of_events_url(std::string_view url) {
  constexpr std::string_view kPrefix = "/api/v1/cases/";
  constexpr std::string_view kSuffix = "/events";
  if (url.rfind(kPrefix, 0) != 0 ||
      url.size() <= kPrefix.size() + kSuffix.size())
    return std::nullopt;
  if (url.substr(url.size() - kSuffix.size()) != kSuffix)
    return std::nullopt;
  const auto id =
      url.substr(kPrefix.size(), url.size() - kPrefix.size() - kSuffix.size());
  if (id.empty() || id.find('/') != std::string_view::npos)
    return std::nullopt;
  return std::string(id);
}

} // namespace ruxd::api
