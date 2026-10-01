// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Link } from 'react-router-dom';

import type { Sample, SurveyType } from '../../api/types';
import { newSampleHref, sampleHref } from '../../app/links';
import { SAMPLE_LINE_LINKED_PREFIX, SAMPLE_LINE_NONE, sampleLineModel } from '../../kortlaegning/samples';
import styles from './SampleLine.module.css';

/**
 * "Miljøstatus styres af …" with each sample a link to its card in Miljø &
 * prøver; for a type with no sample, a link that opens the create form with
 * the type pre-linked. Shared by DetailPanel and EditDialog.
 *
 * The links are plain `<a href>`s: the Kortlægning key maps leave Enter/Space
 * on a link to its native activation (`isControl`), and EditDialog's Tab trap
 * (`a[href]` in its FOCUSABLE) includes them.
 */
export function SampleLine({ type, samples, className }: { type: SurveyType; samples: Sample[]; className?: string }) {
  const m = sampleLineModel(type, samples);
  if (m.kind === 'none') {
    return (
      <p className={className}>
        {SAMPLE_LINE_NONE}{' '}
        <Link className={styles.link} to={newSampleHref(m.typeId)}>
          Registrér prøve
        </Link>
      </p>
    );
  }
  return (
    <p className={className}>
      {SAMPLE_LINE_LINKED_PREFIX}{' '}
      {m.items.map((item, i) => (
        <span key={item.id}>
          {i > 0 && ', '}
          <Link className={styles.link} to={sampleHref(item.id)}>
            {item.text}
          </Link>
        </span>
      ))}
    </p>
  );
}
