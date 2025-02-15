import fs from 'fs';
import path from 'path';

interface CzConfigType {
  value: string;
}

interface CzConfig {
  types: CzConfigType[];
}

// Path to your .cz-config.js file
const czConfigPath: string = path.resolve(__dirname, '.cz-config.js');

// Extract the type values
const czConfig: CzConfig = fs.existsSync(czConfigPath) ? require(czConfigPath) : { types: [] };

const typeValues: string[] = czConfig.types.map((type: CzConfigType) => type.value);

module.exports = {
  extends: ['@commitlint/config-conventional'],

  rules: {
    'type-enum': [
      2,
      'always',
      typeValues, // Use the extracted type values
    ],
    'scope-case': [2, 'always', 'lower-case'],
    'type-case': [2, 'always', 'lower-case'],
  },
};