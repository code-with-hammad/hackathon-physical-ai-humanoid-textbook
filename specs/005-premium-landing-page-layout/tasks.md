# Task Plan: Premium Landing Page Layout

This document outlines the implementation tasks for the **Premium Landing Page Layout** feature, based on the approved specification and implementation plan.

## Implementation Strategy

The implementation will be phased to deliver value incrementally:

1.  **Setup**: Create the necessary files and directories.
2.  **Foundational Data**: Implement the build-time script to generate the navigation data. This is critical for the side navigation.
3.  **UI Implementation**: Develop the React component for the landing page with static content and basic styling.
4.  **Polish**: Apply the final theme, animations, and responsive styles.

This approach ensures the core structure is in place before focusing on aesthetics and advanced features.

---

## Phase 1: Setup

These tasks prepare the project structure for the new feature.

- [x] T001 Create the directory for the new component in `src/components/PremiumLandingPage`

---

## Phase 2: Foundational - Data Generation

This phase focuses on creating the data needed for the side navigation.

- [ ] T002 Implement a script or Docusaurus lifecycle hook to traverse the `docs/` directory and generate a `nav-data.json` file in `src/components/PremiumLandingPage` at build time. The structure should match the `data-model.md`.

---

## Phase 3: UI Implementation

This phase implements the core UI of the landing page, based on User Stories 1 and 2.

**Independent Test**: The landing page can be viewed at `http://localhost:3000`. The centered content is visible, the side navigation is populated from the generated JSON, and the "Start Reading" button navigates to `/docs/intro`.

- [ ] T003 [P] Create the main component file at `src/components/PremiumLandingPage/index.tsx`.
- [ ] T004 [P] Create the stylesheet at `src/components/PremiumLandingPage/styles.module.css`.
- [ ] T005 [US1] Implement the basic two-column layout (centered content, side navigation) in `src/components/PremiumLandingPage/index.tsx` using CSS Modules.
- [ ] T006 [US1] Implement the centered content section, including the book title, a short definition, and the "Start Reading" button.
- [ ] T007 [US2] Ensure the "Start Reading" button in `src/components/PremiumLandingPage/index.tsx` is a `<Link>` component that navigates to `/docs/intro`.
- [ ] T008 [US1] Import the `nav-data.json` and implement the side navigation to render the list of modules and chapters with placeholder icons.
- [ ] T009 Modify the `src/pages/index.tsx` file to import and render the `PremiumLandingPage` component instead of its default content.

---

## Phase 4: Polish & Cross-Cutting Concerns

This phase adds the final visual touches and ensures the page meets all aesthetic and responsive requirements.

- [ ] T010 [P] Propose and define the new theme (color palette, fonts) in `src/css/custom.css`.
- [ ] T011 [US1] Implement a subtle animation or hover effect on the "Start Reading" button using CSS in `src/components/PremiumLandingPage/styles.module.css`.
- [ ] T012 [US1] Refine the responsive styles in `src/components/PremiumLandingPage/styles.module.css` to ensure the layout is optimized for mobile, tablet, and desktop screens.

## Task Dependencies

- **Phase 2** must be completed before **T008**.
- **Phase 3** tasks (T003-T009) can largely be worked on in parallel after Phase 2 is complete, though implementing the layout (T005) is a logical first step.
- **Phase 4** can begin after Phase 3 is functionally complete.
