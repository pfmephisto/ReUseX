// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Password hashing and secret tokens (ruxd server mode, spec 2026-10-08 S3):
// argon2id through OpenSSL's EVP_KDF, the PHC string, token generation and
// the SHA-256 that sessions and API tokens are stored as.

#include <catch2/catch_test_macros.hpp>

#include <api/credentials.hpp>

#include <set>
#include <string>

using namespace ruxd::api;

namespace {
/// Cheap parameters: the tests check behaviour, not cost.
Argon2Params fast() {
  Argon2Params p;
  p.iterations = 1;
  p.memory_kib = 256;
  p.lanes = 1;
  return p;
}
} // namespace

TEST_CASE("Argon2Params_Defaults_FollowRfc9106", "[ruxd_api][auth]") {
  // RFC 9106 §4, second recommended option: 64 MiB, t=3, p=4, 16-byte salt,
  // 32-byte tag. Changing these changes every new hash; old ones still verify.
  const Argon2Params p;
  CHECK(p.iterations == 3);
  CHECK(p.memory_kib == 64u * 1024u);
  CHECK(p.lanes == 4);
  CHECK(p.salt_bytes == 16);
  CHECK(p.hash_bytes == 32);
}

TEST_CASE("HashPassword_PhcString_CarriesItsParameters", "[ruxd_api][auth]") {
  const auto hash = hash_password("correct horse", fast());
  CHECK(hash.rfind("$argon2id$v=19$m=256,t=1,p=1$", 0) == 0);
  const auto parsed = parse_phc(hash);
  REQUIRE(parsed);
  CHECK(parsed->params.memory_kib == 256);
  CHECK(parsed->params.iterations == 1);
  CHECK(parsed->params.lanes == 1);
  CHECK(parsed->salt.size() == 16);
  CHECK(parsed->hash.size() == 32);
  CHECK(format_phc(*parsed) == hash);

  // A fresh salt every time: equal passwords, different hashes.
  CHECK(hash_password("correct horse", fast()) != hash);
}

TEST_CASE("HashPassword_DefaultParameters_RoundTrip", "[ruxd_api][auth]") {
  const auto hash = hash_password("s3cret-Password");
  CHECK(hash.find("$m=65536,t=3,p=4$") != std::string::npos);
  CHECK(verify_password("s3cret-Password", hash));
  CHECK_FALSE(verify_password("s3cret-password", hash));
}

TEST_CASE("VerifyPassword_RightWrongAndMalformed", "[ruxd_api][auth]") {
  const auto hash = hash_password("abc12345", fast());
  CHECK(verify_password("abc12345", hash));
  CHECK_FALSE(verify_password("abc12346", hash));
  CHECK_FALSE(verify_password("", hash));
  CHECK_FALSE(verify_password("abc12345", ""));
  CHECK_FALSE(verify_password("abc12345",
                              "$argon2i$v=19$m=256,t=1,p=1$"
                              "AAAAAAAAAAAAAAAAAAAAAA$"
                              "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
                              "AAAAAA"));
  // Tampered parameters are refused, not run: no 64 GiB allocation.
  CHECK_FALSE(parse_phc("$argon2id$v=19$m=67108864,t=1,p=1$"
                        "AAAAAAAAAAAAAAAAAAAAAA$"
                        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"));
  CHECK_FALSE(parse_phc("$argon2id$v=16$m=256,t=1,p=1$AAAA$AAAA"));
  CHECK_FALSE(parse_phc("$argon2id$v=19$m=256,t=1$AAAAAAAAAAAA$AAAA"));
}

TEST_CASE("PasswordNeedsRehash_WhenParametersChange", "[ruxd_api][auth]") {
  const auto hash = hash_password("abc12345", fast());
  CHECK_FALSE(password_needs_rehash(hash, fast()));
  auto stronger = fast();
  stronger.iterations = 2;
  CHECK(password_needs_rehash(hash, stronger));
  CHECK(password_needs_rehash("not a hash", fast()));
}

TEST_CASE("RandomToken_32Bytes_Base64UrlAndUnique", "[ruxd_api][auth]") {
  std::set<std::string> seen;
  for (int i = 0; i < 64; ++i) {
    const auto token = random_token(32);
    CHECK(token.size() == 43); // 256 bits, unpadded base64url
    CHECK(token.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnop"
                                  "qrstuvwxyz0123456789-_") ==
          std::string::npos);
    seen.insert(token);
  }
  CHECK(seen.size() == 64);
}

TEST_CASE("Sha256Hex_KnownVectorsAndTokenHashing", "[ruxd_api][auth]") {
  CHECK(sha256_hex("") ==
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  CHECK(sha256_hex("abc") ==
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  // A stored session is the digest of its token, never the token.
  const auto token = random_token();
  const auto digest = sha256_hex(token);
  CHECK(digest.size() == 64);
  CHECK(digest != token);
  CHECK(sha256_hex(token) == digest);
}

TEST_CASE("SecureEquals_ConstantTimeComparison", "[ruxd_api][auth]") {
  CHECK(secure_equals("abc", "abc"));
  CHECK_FALSE(secure_equals("abc", "abd"));
  CHECK_FALSE(secure_equals("abc", "abcd"));
  CHECK(secure_equals("", ""));
}

TEST_CASE("Base64Unpadded_RoundTripsEveryLength", "[ruxd_api][auth]") {
  std::string bytes;
  for (int n = 0; n < 40; ++n) {
    const auto encoded = base64_encode_unpadded(bytes);
    CHECK(encoded.find('=') == std::string::npos);
    const auto decoded = base64_decode_unpadded(encoded);
    REQUIRE(decoded);
    CHECK(*decoded == bytes);
    bytes += static_cast<char>(n * 37);
  }
  CHECK_FALSE(base64_decode_unpadded("A"));
  CHECK_FALSE(base64_decode_unpadded("AB*C"));
}
