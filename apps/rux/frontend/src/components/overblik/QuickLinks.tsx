// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Link } from 'react-router-dom';

import type { QuickLink } from '../../overblik/model';
import styles from './QuickLinks.module.css';

export interface QuickLinksProps {
  links: QuickLink[];
}

/** The four case screens, each with what is waiting there. */
export function QuickLinks({ links }: QuickLinksProps) {
  return (
    <nav aria-label="Sagens skærme">
      <ul className={styles.grid}>
        {links.map((l) => (
          <li key={l.to}>
            <Link to={l.to} className={styles.link}>
              <span className={styles.title}>{l.title}</span>
              <span className={styles.sub}>{l.sub}</span>
            </Link>
          </li>
        ))}
      </ul>
    </nav>
  );
}
