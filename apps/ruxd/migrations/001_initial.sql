-- SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
--
-- SPDX-License-Identifier: GPL-3.0-or-later

-- ruxd server mode, first schema (spec 2026-10-08, phase S3).
--
-- Applied by src/pg/migrations.cpp inside one transaction, under an advisory
-- lock, and recorded in schema_migrations. Never edit a migration that has
-- shipped: add NNN_<name>.sql instead.

-- Accounts. Emails are stored lower-cased; the password is an argon2id PHC
-- string (src/api/credentials.cpp), never the password.
CREATE TABLE users (
    id            bigserial PRIMARY KEY,
    email         text        NOT NULL UNIQUE CHECK (email = lower(email)),
    display_name  text        NOT NULL,
    password_hash text        NOT NULL,
    is_admin      boolean     NOT NULL DEFAULT false,
    disabled      boolean     NOT NULL DEFAULT false,
    created_at    timestamptz NOT NULL DEFAULT now()
);

-- Login sessions. Only the SHA-256 of the 32-byte cookie token is stored.
CREATE TABLE sessions (
    token_hash text        PRIMARY KEY,
    user_id    bigint      NOT NULL REFERENCES users (id) ON DELETE CASCADE,
    created_at timestamptz NOT NULL,
    expires_at timestamptz NOT NULL,
    last_seen  timestamptz NOT NULL
);
CREATE INDEX sessions_user_idx ON sessions (user_id);
CREATE INDEX sessions_expires_idx ON sessions (expires_at);

-- Cases: one .rux project each, in its own directory under --data-dir
-- (storage_path is the project file). `slug` is the id in every URL.
CREATE TABLE cases (
    id           bigserial PRIMARY KEY,
    slug         text        NOT NULL UNIQUE,
    name         text        NOT NULL,
    storage_path text        NOT NULL,
    created_by   bigint      REFERENCES users (id) ON DELETE SET NULL,
    created_at   timestamptz NOT NULL DEFAULT now(),
    archived     boolean     NOT NULL DEFAULT false
);

-- API tokens for scripts and CI: hashed like sessions, optionally limited to
-- one case, expiring (NULL = never), revocable by deleting the row.
CREATE TABLE api_tokens (
    id           bigserial PRIMARY KEY,
    token_hash   text        NOT NULL UNIQUE,
    user_id      bigint      NOT NULL REFERENCES users (id) ON DELETE CASCADE,
    name         text        NOT NULL,
    case_id      bigint      REFERENCES cases (id) ON DELETE CASCADE,
    created_at   timestamptz NOT NULL DEFAULT now(),
    expires_at   timestamptz,
    last_used_at timestamptz
);
CREATE INDEX api_tokens_user_idx ON api_tokens (user_id);

CREATE TABLE case_members (
    case_id bigint NOT NULL REFERENCES cases (id) ON DELETE CASCADE,
    user_id bigint NOT NULL REFERENCES users (id) ON DELETE CASCADE,
    role    text   NOT NULL CHECK (role IN ('owner', 'editor', 'viewer')),
    PRIMARY KEY (case_id, user_id)
);
CREATE INDEX case_members_user_idx ON case_members (user_id);

-- Pipeline jobs, saved at each state transition (never per progress tick).
-- `record` is the full JobRecord; the other columns are for queries.
CREATE TABLE jobs (
    id           text        PRIMARY KEY,
    case_id      bigint      NOT NULL REFERENCES cases (id) ON DELETE CASCADE,
    user_id      bigint      REFERENCES users (id) ON DELETE SET NULL,
    stage        text        NOT NULL,
    params       jsonb,
    state        text        NOT NULL,
    submitted_at timestamptz NOT NULL,
    started_at   timestamptz,
    finished_at  timestamptz,
    error        text,
    record       jsonb       NOT NULL
);
CREATE INDEX jobs_case_idx ON jobs (case_id, submitted_at DESC);

-- Who changed what. Written by the server, never read back by it; the case
-- slug is kept so an entry outlives its case.
CREATE TABLE audit_log (
    id        bigserial PRIMARY KEY,
    user_id   bigint      REFERENCES users (id) ON DELETE SET NULL,
    actor     text        NOT NULL,
    case_id   bigint      REFERENCES cases (id) ON DELETE SET NULL,
    case_slug text,
    action    text        NOT NULL,
    detail    text        NOT NULL DEFAULT '',
    at        timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX audit_log_at_idx ON audit_log (at);
