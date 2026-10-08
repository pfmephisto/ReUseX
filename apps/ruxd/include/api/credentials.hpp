// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Password hashing and secret tokens for ruxd's users (spec 2026-10-08,
// phase S3). OpenSSL 3 only: argon2id through EVP_KDF "ARGON2ID", random
// bytes from RAND_bytes, SHA-256 from EVP_Digest, and every comparison of
// secret material through CRYPTO_memcmp.
//
// Framework-free; tested in tests/unit/ruxd_api/test_api_credentials.cpp.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace ruxd::api {

/// argon2id cost parameters. The defaults follow RFC 9106's second
/// recommended option (64 MiB, 3 passes, 4 lanes): about 0.1 s per hash on a
/// workstation core, which bounds an online guesser without making a login
/// noticeably slow. They are written into every hash (PHC string format), so
/// raising them later never breaks an existing password; verify_password()
/// reads them back, and password_needs_rehash() tells a login to upgrade.
struct Argon2Params {
  std::uint32_t iterations = 3;         ///< t: passes over memory.
  std::uint32_t memory_kib = 64 * 1024; ///< m: KiB of memory.
  std::uint32_t lanes = 4;              ///< p: parallelism (lanes).
  std::size_t salt_bytes = 16;          ///< Random salt per hash.
  std::size_t hash_bytes = 32;          ///< Derived key length.

  bool operator==(const Argon2Params &) const = default;
};

/// A parsed `$argon2id$v=19$m=…,t=…,p=…$<salt>$<hash>` string.
struct PhcHash {
  Argon2Params params;
  std::string salt; ///< Raw bytes.
  std::string hash; ///< Raw bytes.
};

/// Parse a PHC argon2id string, or nullopt when it is not one (another
/// algorithm, another version, malformed). Unpadded standard base64, as the
/// reference implementation writes it.
std::optional<PhcHash> parse_phc(std::string_view phc);

/// The PHC string for @p hash.
std::string format_phc(const PhcHash &hash);

/// Hash @p password with a fresh random salt. @return a PHC string.
/// @throws std::runtime_error when OpenSSL has no ARGON2ID KDF (OpenSSL < 3.2)
///         or the derivation fails.
std::string hash_password(std::string_view password,
                          const Argon2Params &params = {});

/// True when @p password matches @p phc. The derived key is compared in
/// constant time. A malformed @p phc never matches.
bool verify_password(std::string_view password, std::string_view phc);

/// True when @p phc was made with weaker (or different) parameters than
/// @p params, so a successful login should store a fresh hash.
bool password_needs_rehash(std::string_view phc, const Argon2Params &params);

/// @p bytes of cryptographically secure randomness, base64url without
/// padding (32 bytes -> 43 characters). @throws std::runtime_error.
std::string random_token(std::size_t bytes = 32);

/// Lower-case hex SHA-256 of @p text. Sessions and API tokens are stored only
/// as this digest of the token, never the token itself.
std::string sha256_hex(std::string_view text);

/// Constant-time equality (CRYPTO_memcmp). Unequal lengths are unequal; the
/// length itself is not secret here (hashes and tokens have fixed lengths).
bool secure_equals(std::string_view a, std::string_view b);

/// Base64 (standard alphabet, no padding) — exposed for the PHC tests.
std::string base64_encode_unpadded(std::string_view bytes);
/// @return nullopt on a character outside the alphabet.
std::optional<std::string> base64_decode_unpadded(std::string_view text);

} // namespace ruxd::api
