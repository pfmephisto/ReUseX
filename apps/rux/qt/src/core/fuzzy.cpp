// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/fuzzy.hpp>

#include <algorithm>
#include <climits>

namespace rux::qt {
namespace {

// Scoring. A matched character is worth kMatch, plus a bonus for where it
// sits (start of the title, start of a word, a camel hump) and for directly
// following the previous match. Gaps cost a little per skipped character,
// capped, so a long title is not punished for a late but well-placed hit.
constexpr int kMatch = 16;
constexpr int kFirstBonus = 40;  // the title's first character
constexpr int kWordBonus = 24;   // after a space or separator
constexpr int kCamelBonus = 16;  // lower -> upper
constexpr int kConsecutive = 20; // follows the previous match directly
constexpr int kGapPerChar = 2;
constexpr int kGapCap = 20;
constexpr int kLeadPerChar = 1;
constexpr int kLeadCap = 10;
constexpr int kFoldPenalty = 2; // "o" matching "ø" is slightly worse than "ø"
// Keyword matches always rank below title matches.
constexpr int kKeywordOffset = -100000;
// Long texts (a keyword path) are only matched within their first part.
constexpr std::size_t kMaxText = 512;
constexpr std::size_t kMaxQuery = 64;
constexpr int kNone = INT_MIN / 4;

std::u32string decode_utf8(std::string_view s) {
  std::u32string out;
  out.reserve(s.size());
  for (std::size_t i = 0; i < s.size();) {
    const auto c = static_cast<unsigned char>(s[i]);
    char32_t cp = 0xFFFD;
    std::size_t len = 1;
    if (c < 0x80) {
      cp = c;
    } else if ((c >> 5) == 0x6 && i + 1 < s.size()) {
      cp = ((c & 0x1Fu) << 6) | (static_cast<unsigned char>(s[i + 1]) & 0x3Fu);
      len = 2;
    } else if ((c >> 4) == 0xE && i + 2 < s.size()) {
      cp = ((c & 0x0Fu) << 12) |
           ((static_cast<unsigned char>(s[i + 1]) & 0x3Fu) << 6) |
           (static_cast<unsigned char>(s[i + 2]) & 0x3Fu);
      len = 3;
    } else if ((c >> 3) == 0x1E && i + 3 < s.size()) {
      cp = ((c & 0x07u) << 18) |
           ((static_cast<unsigned char>(s[i + 1]) & 0x3Fu) << 12) |
           ((static_cast<unsigned char>(s[i + 2]) & 0x3Fu) << 6) |
           (static_cast<unsigned char>(s[i + 3]) & 0x3Fu);
      len = 4;
    }
    out.push_back(cp);
    i += len;
  }
  return out;
}

char32_t lower(char32_t c) {
  if (c >= U'A' && c <= U'Z')
    return c + 32;
  if (c >= 0xC0 && c <= 0xDE && c != 0xD7) // Latin-1 capitals (Æ Ø Å …)
    return c + 0x20;
  return c;
}

bool is_upper(char32_t c) {
  return (c >= U'A' && c <= U'Z') || (c >= 0xC0 && c <= 0xDE && c != 0xD7);
}

/// The ASCII letter a lower-case Latin-1 letter is typed as, or 0.
char32_t base_letter(char32_t c) {
  if (c >= 0xE0 && c <= 0xE6) // à … å, æ
    return U'a';
  if (c == 0xE7)
    return U'c';
  if (c >= 0xE8 && c <= 0xEB)
    return U'e';
  if (c >= 0xEC && c <= 0xEF)
    return U'i';
  if (c == 0xF1)
    return U'n';
  if ((c >= 0xF2 && c <= 0xF6) || c == 0xF8) // ò … ö, ø
    return U'o';
  if (c >= 0xF9 && c <= 0xFC)
    return U'u';
  if (c == 0xFD || c == 0xFF)
    return U'y';
  return 0;
}

bool is_separator(char32_t c) {
  return c == U' ' || c == U'/' || c == U'\\' || c == U'-' || c == U'_' ||
         c == U'.' || c == U'(' || c == U'[' || c == U':' || c == U',';
}

/// kMatch-based score for query char @p q against text char @p t (both
/// lowered), or kNone.
int char_score(char32_t q, char32_t t) {
  if (q == t)
    return kMatch;
  if (q < 0x80 && base_letter(t) == q)
    return kMatch - kFoldPenalty;
  return kNone;
}

int position_bonus(const std::u32string &text, std::size_t j) {
  if (j == 0)
    return kFirstBonus;
  const char32_t prev = text[j - 1];
  if (is_separator(prev))
    return kWordBonus;
  if (is_upper(text[j]) && !is_upper(prev) && !is_separator(prev))
    return kCamelBonus;
  return 0;
}

int gap_cost(std::size_t gap) {
  return -std::min<int>(static_cast<int>(gap) * kGapPerChar, kGapCap);
}

} // namespace

int fuzzy_score(std::string_view query_utf8, std::string_view text_utf8,
                std::vector<int> *positions) {
  std::u32string query;
  for (char32_t c : decode_utf8(query_utf8))
    if (c != U' ')
      query.push_back(lower(c));
  if (query.empty())
    return 0;
  if (query.size() > kMaxQuery)
    query.resize(kMaxQuery);

  std::u32string text = decode_utf8(text_utf8);
  if (text.size() > kMaxText)
    text.resize(kMaxText);
  std::u32string folded(text.size(), 0);
  for (std::size_t j = 0; j < text.size(); ++j)
    folded[j] = lower(text[j]);

  const std::size_t m = query.size();
  const std::size_t n = text.size();
  if (m > n)
    return -1;

  // dp[i][j]: best score with query[0..i] matched and query[i] at text[j].
  std::vector<std::vector<int>> dp(m, std::vector<int>(n, kNone));
  std::vector<std::vector<int>> from(m, std::vector<int>(n, -1));

  for (std::size_t j = 0; j < n; ++j) {
    const int cs = char_score(query[0], folded[j]);
    if (cs == kNone)
      continue;
    dp[0][j] = cs + position_bonus(text, j) -
               std::min<int>(static_cast<int>(j) * kLeadPerChar, kLeadCap);
  }

  for (std::size_t i = 1; i < m; ++i) {
    // Running best of dp[i-1][k] over k far enough back that the gap cost
    // is capped (gap >= kGapCap / kGapPerChar).
    const std::size_t cap_gap = kGapCap / kGapPerChar;
    int far_best = kNone;
    int far_arg = -1;
    for (std::size_t j = i; j < n; ++j) {
      // Admit k = j - 1 - cap_gap into the capped running maximum.
      if (j >= cap_gap + 1) {
        const std::size_t k = j - 1 - cap_gap;
        if (dp[i - 1][k] > far_best) {
          far_best = dp[i - 1][k];
          far_arg = static_cast<int>(k);
        }
      }
      const int cs = char_score(query[i], folded[j]);
      if (cs == kNone)
        continue;
      int best = kNone;
      int arg = -1;
      if (far_best != kNone) {
        best = far_best - kGapCap;
        arg = far_arg;
      }
      // Near predecessors: k in (j - 1 - cap_gap, j - 1].
      const std::size_t lo = j > cap_gap ? j - cap_gap : 0;
      for (std::size_t k = lo; k < j; ++k) {
        if (dp[i - 1][k] == kNone)
          continue;
        const std::size_t gap = j - k - 1;
        const int v = dp[i - 1][k] + (gap == 0 ? kConsecutive : gap_cost(gap));
        if (v > best) {
          best = v;
          arg = static_cast<int>(k);
        }
      }
      if (best == kNone)
        continue;
      dp[i][j] = best + cs + position_bonus(text, j);
      from[i][j] = arg;
    }
  }

  int best = kNone;
  int end = -1;
  for (std::size_t j = 0; j < n; ++j)
    if (dp[m - 1][j] > best) {
      best = dp[m - 1][j];
      end = static_cast<int>(j);
    }
  if (end < 0)
    return -1;
  if (positions) {
    positions->assign(m, 0);
    int j = end;
    for (std::size_t i = m; i-- > 0;) {
      (*positions)[i] = j;
      j = from[i][static_cast<std::size_t>(j)];
    }
  }
  // A real match never scores below 1, so -1 stays "no match".
  return std::max(best, 1);
}

std::vector<PaletteMatch>
rank_palette(std::string_view query,
             const std::vector<PaletteCandidate> &candidates) {
  std::vector<PaletteMatch> out;
  const bool empty = query.find_first_not_of(' ') == std::string_view::npos;
  if (empty) {
    out.reserve(candidates.size());
    for (std::size_t i = 0; i < candidates.size(); ++i)
      out.push_back({i, 0, {}});
    return out;
  }
  std::vector<std::size_t> title_len(candidates.size());
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    title_len[i] = decode_utf8(candidates[i].title).size();
    PaletteMatch m{i, 0, {}};
    const int s = fuzzy_score(query, candidates[i].title, &m.positions);
    if (s >= 0) {
      m.score = s;
      out.push_back(std::move(m));
      continue;
    }
    const int k = fuzzy_score(query, candidates[i].keywords);
    if (k >= 0) {
      m.positions.clear();
      m.score = k + kKeywordOffset;
      out.push_back(std::move(m));
    }
  }
  std::stable_sort(out.begin(), out.end(),
                   [&](const PaletteMatch &a, const PaletteMatch &b) {
                     if (a.score != b.score)
                       return a.score > b.score;
                     return title_len[a.index] < title_len[b.index];
                   });
  return out;
}

} // namespace rux::qt
