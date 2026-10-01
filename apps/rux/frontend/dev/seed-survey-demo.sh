#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Copy a .rux project and seed the prototype-v2 demo survey (Måløv Byvej 229)
# into the copy, for screenshots and manual testing of Kortlægning. Parts are
# not linked to instances (the demo has none), so evidence renders show the
# whole cloud without a highlight. Never run this against a real project.
#
# Usage: seed-survey-demo.sh [--varied] <source.rux> <dest.rux>
# --varied adds P-04 (answered ren, linked to two approved types) and P-05 (planned, unlinked) for Miljø & prøver work.
set -euo pipefail
command -v sqlite3 > /dev/null 2>&1 || { echo "sqlite3 not found" >&2; exit 1; }
varied=0
if [[ "${1:-}" == "--varied" ]]; then varied=1; shift; fi
src="${1:?source .rux}"
dst="${2:?destination .rux}"
[[ -f "$src" ]] || { echo "no such project: $src" >&2; exit 1; }
cp "$src" "$dst"
rm -f "$dst-wal" "$dst-shm"
# Opening the copy read-write once migrates it to the latest schema. `create
# survey` opens read-write; it may fail (no instances) after migrating, which is
# fine — the seed below replaces whatever it wrote.
rux -p "$dst" create survey > /dev/null 2>&1 || true
ver=$(sqlite3 "$dst" 'SELECT MAX(version) FROM schema_version;')
[[ "$ver" -ge 22 ]] || { echo "schema v$ver < 22 — build a newer rux" >&2; exit 1; }
sqlite3 "$dst" <<'SQL'
BEGIN;
DELETE FROM sample_links; DELETE FROM samples; DELETE FROM survey_parts; DELETE FROM survey_types;
INSERT INTO survey_types (id,name,eak_code,bim7aa_code,unit,treatment,review_status,confidence,mass_t,note,starred,semantic_class) VALUES
 (1,'Fundamenter & terrændæk, beton','17.01.01','131 Fundamenter','m³','bevaring','approved',0.94,640,'Bevares in situ — genanvendes i nyt byggeri på grunden.',0,-2),
 (2,'Betonsøjler, bærende','17.01.01','221 Bærende konstr.','stk','genbrug','queue',0.88,58,'Præfab søjler i god stand — direkte genbrug ved dokumenteret bæreevne.',0,-2),
 (3,'Betondæk, etagedæk','17.01.01','231 Etagedæk','m²','genanvendelse','queue',0.91,380,'Nedknuses til vejfyld/ny beton. Største enkeltfraktion.',0,-2),
 (4,'Facadeelementer, sandwich','17.01.01','211 Ydervægge','m²','genanvendelse','approved',0.90,190,'Elementsamlinger tillader hel nedtagning — afsætning undersøges.',0,-2),
 (5,'Stålspær, tagkonstruktion','17.04.05','271 Tagkonstruktion','stk','genbrug','queue',0.93,14,'Ensartede spær, boltede samlinger — høj genbrugsværdi.',1,-2),
 (6,'Vinduespartier, aluminium','17.04.02','312 Udv. vinduer','stk','genbrug','queue',0.82,3.1,'Ved ren fuge: salg som brugte partier.',0,-2),
 (7,'Trapezplader, tag','17.04.05','272 Tagdækning','m²','genanvendelse','approved',0.89,6.8,'Skrot/omsmeltning via metalgenvinding.',0,-2),
 (8,'Gulvbelægning, linoleum','17.09.04','421 Gulvbelægning','m²','nyttiggoerelse','queue',0.84,1.6,'Behandling afhænger af limprøve (asbest).',0,-2),
 (9,'Indvendige døre, træ','17.02.01','322 Indv. døre','stk','genbrug','queue',0.74,0.9,'Blandet stand — ★ fra on-site: 8–10 stk. skønnes direkte genbrugelige.',1,-2),
 (10,'Isolering, mineraluld','17.06.04','251 Isolering','m²','bortskaffelse','approved',0.87,2.4,'Deponi medmindre retur-ordning kan afsætte.',0,-2),
 (11,'Indvendige murvægge, malet','17.01.02','222 Indervægge','m²','bortskaffelse','queue',0.87,38,'Bly i maling påvist (P-02) — afrenses eller håndteres som forurenet.',0,-2);
INSERT INTO survey_parts (code,type_id,instance_guid,room_id,room_name,quantity) VALUES
 ('RX-010',1,NULL,1,'Production Hall',290),('RX-011',1,NULL,2,'Office Zone',30),
 ('RX-001',2,NULL,1,'Production Hall',18),('RX-002',2,NULL,4,'Entrance',6),
 ('RX-003',3,NULL,1,'Production Hall',980),('RX-004',3,NULL,2,'Office Zone',260),
 ('RX-005',4,NULL,6,'Facade',340),('RX-006',4,NULL,6,'Facade',280),
 ('RX-007',5,NULL,5,'Roof',22),
 ('RX-008',6,NULL,2,'Office Zone',26),('RX-009',6,NULL,1,'Production Hall',12),
 ('RX-012',7,NULL,5,'Roof',780),
 ('RX-013',8,NULL,2,'Office Zone',310),
 ('RX-014',9,NULL,2,'Office Zone',14),('RX-015',9,NULL,1,'Production Hall',10),
 ('RX-016',10,NULL,5,'Roof',480),
 ('RX-017',11,NULL,1,'Production Hall',170),('RX-018',11,NULL,3,'Technical Room',70);
INSERT INTO samples (id,code,title,what,stage,result) VALUES
 (1,'P-01','PCB i fugemasse','Fugemasse omkring vinduespartier','sendt',''),
 (2,'P-02','Bly i maling','Malede indervægge, Production Hall + Technical Room','svar','forurenet'),
 (3,'P-03','Asbest i linoleumslim','Gulvlim under linoleum, Office Zone','udtaget','');
INSERT INTO sample_links (sample_id,type_id) VALUES (1,6),(2,11),(3,8);
COMMIT;
SQL
if [[ "$varied" -eq 1 ]]; then
sqlite3 "$dst" <<'SQL'
BEGIN;
INSERT INTO samples (id,code,title,what,stage,result) VALUES
 (4,'P-04','Asbest i eternitplader','Tagplader over Roof, prøve fra nordfaldet','svar','ren'),
 (5,'P-05','PAH i tagpap','Tagpap under trapezplader — endnu ikke udtaget','planlagt','');
INSERT INTO sample_links (sample_id,type_id) VALUES (4,7),(4,10);
COMMIT;
SQL
fi
echo "seeded demo survey into $dst"
