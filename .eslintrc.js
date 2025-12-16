module.exports = {
  root: true,
  env: {
    browser: true,
    es2021: true,
    node: true,
  },
  extends: [
    'eslint:recommended',
    'plugin:react/recommended',
    'plugin:@typescript-eslint/recommended',
    'prettier', // Make sure this is always the last configuration in the extends array.
  ],
  parser: '@typescript-eslint/parser',
  parserOptions: {
    ecmaFeatures: {
      jsx: true,
    },
    ecmaVersion: 12,
    sourceType: 'module',
  },
  plugins: ['react', '@typescript-eslint'],
  rules: {
    'react/react-in-jsx-scope': 'off', // React 17+ doesn't require React to be in scope
    '@typescript-eslint/explicit-module-boundary-types': 'off', // Allow implicit return types
    '@typescript-eslint/no-var-requires': 'off', // Allow require statements
    'no-unused-vars': 'off', // Handled by TypeScript's noUnusedLocals / noUnusedParameters
    '@typescript-eslint/no-unused-vars': ['warn', { argsIgnorePattern: '^_' }],
  },
  settings: {
    react: {
      version: 'detect', // Automatically detect the React version
    },
  },
  ignorePatterns: [
    'node_modules/',
    'build/',
    'dist/',
    '.docusaurus/',
    'src/css/',
  ],
};
