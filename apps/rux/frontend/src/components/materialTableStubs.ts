// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Local stubs for the material-editor front-end (#416).
 *
 * The backend half (#413/#414/#415) owns `api/types.ts` and `api/client.ts`
 * and has not merged yet, so the property-definition schema and the column /
 * row / thumbnail endpoints live here, isolated in one file. When the backend
 * PR lands, this file is deleted: the types move to `types.ts` imports and the
 * `apiStubs` calls become `api.*` calls. Everything shared by
 * `MaterialTable`, `EditableCell` and `ColumnHeaderMenu` is kept here so there
 * is a single seam to cut.
 */

export type PropertyType = 'text' | 'number' | 'date' | 'boolean' | 'select';

export interface PropertyDefinition {
  id: string;
  name: string;
  type: PropertyType;
  options?: string[];
  sort_order: number;
}

/** The five property types, in the order the header menu offers them. */
export const PROPERTY_TYPES: PropertyType[] = ['text', 'number', 'date', 'boolean', 'select'];

/** Display labels for the property types. */
export const PROPERTY_TYPE_LABELS: Record<PropertyType, string> = {
  text: 'Text',
  number: 'Number',
  date: 'Date',
  boolean: 'Checkbox',
  select: 'Select',
};

// Stub API calls — replace with api.* when backend PR merges (#413/#414/#415).
export const apiStubs = {
  propertyDefinitions: async (): Promise<PropertyDefinition[]> => [],
  createMaterial: async () => {
    console.warn('createMaterial: backend stub');
  },
  deleteMaterial: async (_guid: string) => {
    console.warn('deleteMaterial stub', _guid);
  },
  createPropertyDefinition: async (_def: Omit<PropertyDefinition, 'id'>) => {
    console.warn('createPropertyDefinition stub');
  },
  updatePropertyDefinition: async (_id: string, _patch: Partial<PropertyDefinition>) => {
    console.warn('updatePropertyDefinition stub', _id);
  },
  deletePropertyDefinition: async (_id: string) => {
    console.warn('deletePropertyDefinition stub', _id);
  },
};
