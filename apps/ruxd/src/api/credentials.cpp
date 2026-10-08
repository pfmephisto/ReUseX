// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/credentials.hpp"

#include <openssl/core_names.h>
#include <openssl/crypto.h>
#include <openssl/evp.h>
#include <openssl/kdf.h>
#include <openssl/params.h>
#include <openssl/rand.h>

#include <array>
#include <charconv>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace ruxd::api {
namespace {

constexpr std::string_view kB64 =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// argon2 version 1.3, the only one written or accepted.
constexpr std::uint32_t kArgon2Version = 0x13;

/// Bounds a stored hash may claim before it is refused as malformed: a
/// tampered row must not make a login allocate gigabytes or spin for minutes.
constexpr std::uint32_t kMaxMemoryKib = 4u * 1024 * 1024; // 4 GiB
constexpr std::uint32_t kMaxIterations = 64;
constexpr std::uint32_t kMaxLanes = 64;

struct KdfDeleter {
  void operator()(EVP_KDF *kdf) const { EVP_KDF_free(kdf); }
  void operator()(EVP_KDF_CTX *ctx) const { EVP_KDF_CTX_free(ctx); }
};

std::string argon2id(std::string_view password, std::string_view salt,
                     const Argon2Params &p, std::size_t out_bytes) {
  std::unique_ptr<EVP_KDF, KdfDeleter> kdf(
      EVP_KDF_fetch(nullptr, "ARGON2ID", nullptr));
  if (!kdf)
    throw std::runtime_error("OpenSSL has no ARGON2ID KDF (needs OpenSSL 3.2 "
                             "or newer)");
  std::unique_ptr<EVP_KDF_CTX, KdfDeleter> ctx(EVP_KDF_CTX_new(kdf.get()));
  if (!ctx)
    throw std::runtime_error("EVP_KDF_CTX_new failed");

  std::uint32_t iterations = p.iterations;
  std::uint32_t memory = p.memory_kib;
  std::uint32_t lanes = p.lanes;
  std::uint32_t threads = 1; // >1 needs OSSL_set_max_threads; lanes still count
  std::uint32_t version = kArgon2Version;
  // OSSL_PARAM takes non-const pointers; nothing is written through them.
  std::string pass(password);
  std::string salt_copy(salt);
  const std::array<OSSL_PARAM, 8> params{{
      OSSL_PARAM_construct_octet_string(OSSL_KDF_PARAM_PASSWORD, pass.data(),
                                        pass.size()),
      OSSL_PARAM_construct_octet_string(OSSL_KDF_PARAM_SALT, salt_copy.data(),
                                        salt_copy.size()),
      OSSL_PARAM_construct_uint32(OSSL_KDF_PARAM_ITER, &iterations),
      OSSL_PARAM_construct_uint32(OSSL_KDF_PARAM_ARGON2_MEMCOST, &memory),
      OSSL_PARAM_construct_uint32(OSSL_KDF_PARAM_ARGON2_LANES, &lanes),
      OSSL_PARAM_construct_uint32(OSSL_KDF_PARAM_THREADS, &threads),
      OSSL_PARAM_construct_uint32(OSSL_KDF_PARAM_ARGON2_VERSION, &version),
      OSSL_PARAM_construct_end(),
  }};
  std::string out(out_bytes, '\0');
  const int ok =
      EVP_KDF_derive(ctx.get(), reinterpret_cast<unsigned char *>(out.data()),
                     out.size(), params.data());
  OPENSSL_cleanse(pass.data(), pass.size());
  if (ok != 1)
    throw std::runtime_error("argon2id derivation failed");
  return out;
}

std::optional<std::uint32_t> parse_u32(std::string_view text) {
  std::uint32_t value = 0;
  const auto *end = text.data() + text.size();
  const auto [ptr, ec] = std::from_chars(text.data(), end, value);
  if (ec != std::errc{} || ptr != end || text.empty())
    return std::nullopt;
  return value;
}

/// Split @p text on '$' (the leading '$' gives an empty first field).
std::vector<std::string_view> split_dollar(std::string_view text) {
  std::vector<std::string_view> out;
  std::size_t start = 0;
  for (;;) {
    const auto pos = text.find('$', start);
    out.push_back(text.substr(start, pos - start));
    if (pos == std::string_view::npos)
      break;
    start = pos + 1;
  }
  return out;
}

} // namespace

std::string base64_encode_unpadded(std::string_view bytes) {
  std::string out;
  out.reserve((bytes.size() * 4 + 2) / 3);
  std::size_t i = 0;
  const auto *data = reinterpret_cast<const unsigned char *>(bytes.data());
  for (; i + 3 <= bytes.size(); i += 3) {
    const std::uint32_t v = (data[i] << 16) | (data[i + 1] << 8) | data[i + 2];
    out += kB64[(v >> 18) & 63];
    out += kB64[(v >> 12) & 63];
    out += kB64[(v >> 6) & 63];
    out += kB64[v & 63];
  }
  const std::size_t rest = bytes.size() - i;
  if (rest == 1) {
    const std::uint32_t v = data[i] << 16;
    out += kB64[(v >> 18) & 63];
    out += kB64[(v >> 12) & 63];
  } else if (rest == 2) {
    const std::uint32_t v = (data[i] << 16) | (data[i + 1] << 8);
    out += kB64[(v >> 18) & 63];
    out += kB64[(v >> 12) & 63];
    out += kB64[(v >> 6) & 63];
  }
  return out;
}

std::optional<std::string> base64_decode_unpadded(std::string_view text) {
  if (text.size() % 4 == 1)
    return std::nullopt;
  std::string out;
  std::uint32_t buffer = 0;
  int bits = 0;
  for (const char c : text) {
    const auto pos = kB64.find(c);
    if (pos == std::string_view::npos)
      return std::nullopt;
    buffer = (buffer << 6) | static_cast<std::uint32_t>(pos);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out += static_cast<char>((buffer >> bits) & 0xff);
    }
  }
  return out;
}

std::optional<PhcHash> parse_phc(std::string_view phc) {
  // "", "argon2id", "v=19", "m=..,t=..,p=..", salt, hash
  const auto fields = split_dollar(phc);
  if (fields.size() != 6 || !fields[0].empty() || fields[1] != "argon2id" ||
      fields[2] != "v=19")
    return std::nullopt;

  PhcHash out;
  bool have_m = false, have_t = false, have_p = false;
  std::string_view list = fields[3];
  while (!list.empty()) {
    const auto comma = list.find(',');
    const auto item = list.substr(0, comma);
    list = comma == std::string_view::npos ? std::string_view{}
                                           : list.substr(comma + 1);
    if (item.size() < 3 || item[1] != '=')
      return std::nullopt;
    const auto value = parse_u32(item.substr(2));
    if (!value)
      return std::nullopt;
    switch (item[0]) {
    case 'm':
      out.params.memory_kib = *value;
      have_m = true;
      break;
    case 't':
      out.params.iterations = *value;
      have_t = true;
      break;
    case 'p':
      out.params.lanes = *value;
      have_p = true;
      break;
    default:
      return std::nullopt;
    }
  }
  if (!have_m || !have_t || !have_p)
    return std::nullopt;
  const auto &p = out.params;
  if (p.iterations < 1 || p.iterations > kMaxIterations || p.lanes < 1 ||
      p.lanes > kMaxLanes || p.memory_kib < 8 * p.lanes ||
      p.memory_kib > kMaxMemoryKib)
    return std::nullopt;

  auto salt = base64_decode_unpadded(fields[4]);
  auto hash = base64_decode_unpadded(fields[5]);
  if (!salt || !hash || salt->size() < 8 || hash->size() < 16 ||
      hash->size() > 64)
    return std::nullopt;
  out.salt = std::move(*salt);
  out.hash = std::move(*hash);
  out.params.salt_bytes = out.salt.size();
  out.params.hash_bytes = out.hash.size();
  return out;
}

std::string format_phc(const PhcHash &hash) {
  return "$argon2id$v=19$m=" + std::to_string(hash.params.memory_kib) +
         ",t=" + std::to_string(hash.params.iterations) +
         ",p=" + std::to_string(hash.params.lanes) + "$" +
         base64_encode_unpadded(hash.salt) + "$" +
         base64_encode_unpadded(hash.hash);
}

std::string hash_password(std::string_view password,
                          const Argon2Params &params) {
  if (params.salt_bytes < 8 || params.hash_bytes < 16 || params.hash_bytes > 64)
    throw std::invalid_argument("argon2id: salt must be at least 8 bytes and "
                                "the hash 16-64 bytes");
  PhcHash out;
  out.params = params;
  out.salt.resize(params.salt_bytes);
  if (RAND_bytes(reinterpret_cast<unsigned char *>(out.salt.data()),
                 static_cast<int>(out.salt.size())) != 1)
    throw std::runtime_error("RAND_bytes failed");
  out.hash = argon2id(password, out.salt, params, params.hash_bytes);
  return format_phc(out);
}

bool verify_password(std::string_view password, std::string_view phc) {
  const auto parsed = parse_phc(phc);
  if (!parsed)
    return false;
  try {
    const std::string derived =
        argon2id(password, parsed->salt, parsed->params, parsed->hash.size());
    return secure_equals(derived, parsed->hash);
  } catch (const std::exception &) {
    return false;
  }
}

bool password_needs_rehash(std::string_view phc, const Argon2Params &params) {
  const auto parsed = parse_phc(phc);
  if (!parsed)
    return true;
  const auto &p = parsed->params;
  return p.iterations != params.iterations ||
         p.memory_kib != params.memory_kib || p.lanes != params.lanes ||
         parsed->salt.size() != params.salt_bytes ||
         parsed->hash.size() != params.hash_bytes;
}

std::string random_token(std::size_t bytes) {
  std::string raw(bytes, '\0');
  if (RAND_bytes(reinterpret_cast<unsigned char *>(raw.data()),
                 static_cast<int>(raw.size())) != 1)
    throw std::runtime_error("RAND_bytes failed");
  std::string out = base64_encode_unpadded(raw);
  OPENSSL_cleanse(raw.data(), raw.size());
  for (char &c : out) // base64url
    if (c == '+')
      c = '-';
    else if (c == '/')
      c = '_';
  return out;
}

std::string sha256_hex(std::string_view text) {
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0;
  if (EVP_Digest(text.data(), text.size(), digest.data(), &length, EVP_sha256(),
                 nullptr) != 1)
    throw std::runtime_error("SHA-256 failed");
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out;
  out.reserve(length * 2);
  for (unsigned int i = 0; i < length; ++i) {
    out += kHex[digest[i] >> 4];
    out += kHex[digest[i] & 15];
  }
  return out;
}

bool secure_equals(std::string_view a, std::string_view b) {
  return a.size() == b.size() &&
         CRYPTO_memcmp(a.data(), b.data(), a.size()) == 0;
}

} // namespace ruxd::api
