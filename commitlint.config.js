module.exports = {
  rules: {
    'header-pattern': [
      2,
      'always',
      /^(?:[a-z]+(?:\([a-z]+\))?: .+|[a-z]+: .+)$/,
    ],
    'header-max-length': [2, 'always', 100], // Max 100 characters for summary
    'subject-empty': [2, 'never'], // Ensure message exists
    'type-case': [2, 'always', 'lower-case'], // Type must be lowercase
    'type-empty': [2, 'never'], // Type must exist
    'scope-case': [2, 'always', 'lower-case'], // Scope must be lowercase if present
    'scope-empty': [0], // Scope is optional
    'subject-case': [0], // Allow any case for the subject
  },
};
