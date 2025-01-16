const subjectCharacterLimit = 100;

module.exports = {
  types: [
    { value: 'feat', name: 'feat:     A new feature' },
    { value: 'fix', name: 'fix:      A bug fix' },
    { value: 'docs', name: 'docs:     Documentation only changes' },
    { value: 'tune', name: 'tune:     Changes that affect the tuning of the robot' },
    {
      value: 'style',
      name: 'style:    Changes that do not affect the meaning of the code\n            (white-space, formatting, missing semi-colons, etc)',
    },
    {
      value: 'refactor',
      name: 'refactor: A code change that neither fixes a bug nor adds a feature',
    },
    {
      value: 'perf',
      name: 'perf:     A code change that improves performance',
    },
    { value: 'test', name: 'test:     Adding missing tests' },
    {
      value: 'chore',
      name: 'chore:    Changes to the build process or auxiliary tools\n            and libraries such as documentation generation',
    },
    { value: 'wip', name: 'wip:      Work in progress' },
    { value: 'revert', name: 'revert:   Revert to a prior commit' },
    { value: 'removal', name: 'removal:  Remove a feature or code section' },
    { value: 'update', name: 'update:   Update a dependency or vendor dep' },
    { value: 'rename', name: 'rename:   Rename a class or function (or something else)' },
  ],

  scopes: [{ name: 'robot' }, { name: 'swerve' }, { name: 'elevator' }, { name: 'pivot' }, { name: 'led' }, { name: 'photon' }, { name: 'robotparams' }, { name: 'command' }, { name: 'utils' }, { name: 'project' }],

  usePreparedCommit: false, // to re-use commit from ./.git/COMMIT_EDITMSG
  allowTicketNumber: false,
  isTicketNumberRequired: false,
  ticketNumberPrefix: 'TICKET-',
  ticketNumberRegExp: '\\d{1,5}',

  // it needs to match the value for field type. Eg.: 'chore'
  scopeOverrides: {
    chore: [
      { name: 'github' },
      { name: 'gradle' },
      { name: 'dokka' },
      { name: 'cz' },
      { name: 'commitlint' },
      { name: 'husky' }
    ],
    tune: [
      { name: 'swerve' },
      { name: 'pivot' },
      { name: 'elevator' }
    ],
    update: []
  },
  // override the messages, defaults are as follows
  messages: {
    type: "Select the type of change that you're committing:",
    scope: '\nDenote the SCOPE of this change (optional):',
    // used if allowCustomScopes is true
    customScope: 'Denote the SCOPE of this change:',
    subject: `Write a SHORT, IMPERATIVE tense description of the change (less than ${subjectCharacterLimit} characters long):\n`,
    body: 'Provide a LONGER description of the change (optional). Use "|" to break new line:\n',
    breaking: 'List any BREAKING CHANGES (optional):\n',
    footer: 'List any ISSUES CLOSED by this change (optional). E.g.: #31, #34:\n',
    confirmCommit: 'Are you sure you want to proceed with the commit above?',
  },

  allowCustomScopes: true,
  allowBreakingChanges: ['feat', 'fix'],
  // skip any questions you want
  // skipQuestions: ['scope', 'body'],

  // limit subject length
  subjectLimit: subjectCharacterLimit,
  breaklineChar: '|', // It is supported for fields body and footer.
  // footerPrefix : 'ISSUES CLOSED:'
  // askForBreakingChangeFirst : true, // default is false
};
