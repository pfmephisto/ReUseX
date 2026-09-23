// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { MaterialTable } from '../components/MaterialTable';

/**
 * Material passports at their own top-level route.
 *
 * Promoted from the old DataPage tab model (issue #451). MaterialTable manages
 * its own full-height layout, so no page wrapper is needed here.
 */
export function MaterialsPage() {
  return <MaterialTable />;
}
