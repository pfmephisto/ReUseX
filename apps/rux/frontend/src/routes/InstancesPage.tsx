// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { InstancePanel } from '../components/InstanceList';

/**
 * Instance list at its own top-level route (`/instances`).
 *
 * Shows all instance records for each semantic label cloud, with a
 * "Create material" action on each unlinked instance.  The action
 * mints a blank material passport, links it to the instance, and
 * prefills its thumbnail from the best source frame before navigating
 * to `/materials` (issue #455).
 */
export function InstancesPage() {
  return <InstancePanel />;
}
