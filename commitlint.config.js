module.exports = {
  extends: ['@commitlint/config-conventional'], // Extend conventional commit types

  rules: {
    'type-enum': [
      2,
      'always',
      [
        'feat', 'fix', 'docs', 'tune', 'style', 'refactor', 'perf', 'test',
        'chore', 'WIP', 'revert', 'removal', 'update',
      ],
    ],
    'scope-case': [2, 'always', 'lower-case'], // Ensures scope is lowercase
    'type-case': [2, 'always', 'lower-case'], // Ensures type is lowercase
  },
};
