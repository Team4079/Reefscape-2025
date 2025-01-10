// import fs from 'fs';
import path from 'path';

// Path to your .cz-config.js file
const czConfigPath = path.resolve(__dirname, '.cz-config.js');

// Read and parse the .cz-config.js file
const czConfig = require(czConfigPath);

// Extract the type values
const typeValues = czConfig.types.map(type => type.value);

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
