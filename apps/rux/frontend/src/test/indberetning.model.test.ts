// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  canSend,
  footStatus,
  fractionRows,
  fractionsCsv,
  fractionsCsvHref,
  SEND_NOTICE,
  tonnesText,
  type BlockingRow,
  type ReadyRow,
} from '../indberetning/model';
import { blockingType, fraction, surveyFractions } from './surveyFixtures';

describe('fraction rows', () => {
  it('lists the ready fractions, then each blocking type with its reason', () => {
    const rows = fractionRows(surveyFractions());
    const ready = rows.filter((r): r is ReadyRow => r.kind === 'ready');
    expect(ready.map((r) => [r.eak, r.fraction, r.treatment, r.amount])).toEqual([
      ['17.01.01', 'Beton', 'Genanvendelse', '190 t'],
      ['17.04.05', 'Jern og stål', 'Genanvendelse', '6,8 t'],
      ['17.06.04', 'Isoleringsmateriale', 'Bortskaffelse', '2,4 t'],
    ]);
    const blocking = rows.filter((r): r is BlockingRow => r.kind === 'blocking');
    expect(rows.indexOf(blocking[0])).toBe(3); // blocking rows come last
    expect(blocking).toHaveLength(7);
    expect(blocking[0]).toMatchObject({
      typeId: 2,
      name: 'Betonsøjler, bærende',
      treatment: 'Genbrug',
      amount: '(58 t)',
      href: '/kortlaegning?type=2',
      status: { tone: 'warn', text: 'Afventer gennemsyn' },
    });
    expect(blocking.find((b) => b.typeId === 6)?.status).toEqual({ tone: 'wait', text: 'Afventer prøvesvar' });
  });

  it('words each blocking reason', () => {
    const rows = fractionRows(
      surveyFractions({
        fractions: [],
        blocking: [
          blockingType({ type_id: 1, reason: 'sample' }),
          blockingType({ type_id: 2, reason: 'review' }),
          blockingType({ type_id: 3, reason: 'mass', mass_t: null }),
        ],
        blocking_types: 3,
      }),
    );
    expect(rows.map((r) => (r.kind === 'blocking' ? r.status.text : ''))).toEqual([
      'Afventer prøvesvar',
      'Afventer gennemsyn',
      'Mangler tons',
    ]);
  });

  it('flags contaminated tonnes and words the gaps', () => {
    const rows = fractionRows(
      surveyFractions({
        fractions: [
          fraction({ eak_code: '17.01.02', name: 'Mursten', treatment: 'bortskaffelse', mass_t: 38, contaminated: true }),
          fraction({ eak_code: '99.99.99', name: '' }),
        ],
        blocking: [blockingType({ mass_t: null, eak_code: '' })],
        blocking_types: 1,
      }),
    );
    expect(rows[0]).toMatchObject({ kind: 'ready', contaminated: true, amount: '38 t' });
    expect(rows[1]).toMatchObject({ kind: 'ready', fraction: 'Ukendt EAK-kode' });
    expect(rows[2]).toMatchObject({ kind: 'blocking', eak: '—', amount: '(—)' });
  });

  it('keeps a contaminated fraction apart from the clean one with the same code', () => {
    const rows = fractionRows(
      surveyFractions({
        fractions: [fraction(), fraction({ contaminated: true, mass_t: 4 })],
        blocking: [],
        blocking_types: 0,
        ready: true,
      }),
    );
    expect(new Set(rows.map((r) => r.key)).size).toBe(2);
  });
});

describe('footer and totals', () => {
  it('says ready, or how many types block', () => {
    expect(footStatus(surveyFractions())).toEqual({ tone: 'warn', text: '7 typer blokerer' });
    expect(footStatus(surveyFractions({ blocking_types: 1 }))).toEqual({ tone: 'warn', text: '1 type blokerer' });
    expect(footStatus(surveyFractions({ ready: true, blocking: [], blocking_types: 0 }))).toEqual({
      tone: 'good',
      text: 'Klar til afsendelse',
    });
    expect(tonnesText(199.2)).toBe('199,2 t');
    expect(tonnesText(1334.8)).toBe('1.334,8 t');
  });
});

describe('the send gate and the CSV', () => {
  it('opens the send gate only when nothing blocks', () => {
    expect(canSend(surveyFractions())).toBe(false);
    expect(canSend(surveyFractions({ ready: true, blocking: [], blocking_types: 0 }))).toBe(true);
    // The list is the evidence: a stale `ready` never opens the gate over a blocker.
    expect(canSend(surveyFractions({ ready: true }))).toBe(false);
    expect(canSend(surveyFractions({ ready: false, blocking: [], blocking_types: 0 }))).toBe(true);
  });

  it('holds only the ready fractions, Danish Excel style', () => {
    expect(fractionsCsv(surveyFractions())).toBe(
      'EAK-kode;Fraktion;Behandling;Forurenet;Mængde (t)\r\n' +
        '17.01.01;Beton;Genanvendelse;Nej;190\r\n' +
        '17.04.05;Jern og stål;Genanvendelse;Nej;6,8\r\n' +
        '17.06.04;Isoleringsmateriale;Bortskaffelse;Nej;2,4\r\n',
    );
  });

  it('writes the server tonnes as they came, without grouping', () => {
    const csv = fractionsCsv(surveyFractions({ fractions: [fraction({ mass_t: 1334.123456 })] }));
    expect(csv.split('\r\n')[1]).toBe('17.01.01;Beton;Genanvendelse;Nej;1334,123456');
  });

  it('quotes a field with a separator, a quote or a line break', () => {
    const csv = fractionsCsv(surveyFractions({ fractions: [fraction({ name: 'Beton; "knust"', contaminated: true })] }));
    expect(csv.split('\r\n')[1]).toBe('17.01.01;"Beton; ""knust""";Genanvendelse;Ja;190');
    const nl = fractionsCsv(surveyFractions({ fractions: [fraction({ name: 'Beton\nknust' })] }));
    expect(nl).toContain('17.01.01;"Beton\nknust";Genanvendelse;Nej;190\r\n');
  });

  it('neutralises a text field Excel would run as a formula', () => {
    const row = (name: string, eak = '17.01.01') =>
      fractionsCsv(surveyFractions({ fractions: [fraction({ name, eak_code: eak })] })).split('\r\n')[1];
    expect(row('=HYPERLINK("http://x","klik")')).toBe(
      '17.01.01;"\'=HYPERLINK(""http://x"",""klik"")";Genanvendelse;Nej;190',
    );
    expect(row('-2+3')).toBe("17.01.01;'-2+3;Genanvendelse;Nej;190");
    expect(row('+x')).toBe("17.01.01;'+x;Genanvendelse;Nej;190");
    expect(row('@SUM(A1)')).toBe("17.01.01;'@SUM(A1);Genanvendelse;Nej;190");
    expect(row('\tx')).toBe("17.01.01;'\tx;Genanvendelse;Nej;190");
    expect(row('\rx')).toBe('17.01.01;"\'\rx";Genanvendelse;Nej;190');
    expect(row('Beton', '=1+1')).toBe("'=1+1;Beton;Genanvendelse;Nej;190");
    expect(row('Beton')).toBe('17.01.01;Beton;Genanvendelse;Nej;190');
  });

  it('downloads with a byte-order mark so Excel reads æøå', () => {
    const href = fractionsCsvHref(surveyFractions());
    expect(href.startsWith('data:text/csv;charset=utf-8,')).toBe(true);
    const body = decodeURIComponent(href.slice(href.indexOf(',') + 1));
    expect(body.charCodeAt(0)).toBe(0xfeff);
    expect(body.slice(1)).toBe(fractionsCsv(surveyFractions()));
  });

  it('says plainly that nothing was sent', () => {
    expect(SEND_NOTICE).toBe(
      'Ikke sendt. Direkte indberetning til bygningsaffald.dk er ikke koblet på endnu — hent tallene som CSV og indtast dem i portalen.',
    );
  });
});
