# Quickstart: Premium Landing Page Layout

This document outlines the steps to quickly set up and view the premium landing page layout within the Docusaurus project.

## Prerequisites

- Node.js (LTS version)
- npm or yarn

## Setup and Run

1.  **Navigate to the project root**:

    ```bash
    cd /path/to/your/finalhackathon
    ```

2.  **Install dependencies**:
    If you haven't already, install the project dependencies:

    ```bash
    npm install
    # or
    yarn install
    ```

3.  **Start the Docusaurus development server**:

    ```bash
    npm run start
    # or
    yarn start
    ```

4.  **Access the Landing Page**:
    Once the development server starts, open your web browser and navigate to `http://localhost:3000` (or the address provided in your terminal). The premium landing page is configured as the main entry point (`src/pages/index.tsx`) and should be visible immediately.

## Testing

To run the frontend tests (unit and component tests for React components):

1.  **Ensure dependencies are installed** (step 2 above).
2.  **Execute tests**:
    ```bash
    npm test
    # or
    yarn test
    ```
    (Note: This assumes Jest and React Testing Library are configured for the project.)
