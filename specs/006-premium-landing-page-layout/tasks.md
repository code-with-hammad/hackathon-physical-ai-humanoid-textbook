# Tasks: Premium Landing Page Layout

**Input**: Design documents from `/specs/006-premium-landing-page-layout/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure per implementation plan (src/components/LandingPage, src/components/Shared)
- [x] T002 Initialize TypeScript, React (Docusaurus) project with CSS Modules dependencies
- [x] T003 [P] Configure ESLint/Prettier for TypeScript/React
- [x] T004 [P] Configure Jest and React Testing Library

---

## Phase 2: User Story 1 - View Premium Landing Page (Priority: P1) 🎯 MVP

**Goal**: Implement and test the visually striking, premium, and mobile-friendly landing page layout.

**Independent Test**: Navigate to the landing page URL on various devices and interact with the primary elements. Verify visual design, responsiveness, and button functionality.

### Tests for User Story 1

- [x] T005 [P] [US1] Unit test for `LandingPage.tsx` component in `src/components/LandingPage/LandingPage.test.tsx` (using Jest and React Testing Library)
- [x] T006 [US1] Manual browser testing for responsive layout and visual design verification (integration test)

### Implementation for User Story 1

- [x] T007 [US1] Create `src/components/LandingPage/LandingPage.tsx` for the main component
- [x] T008 [P] [US1] Create `src/components/LandingPage/LandingPage.module.css` for styling
- [x] T009 [P] [US1] Create `src/components/LandingPage/LandingPage.animations.ts` for button animations
- [x] T010 [US1] Implement central section (bold title, definition, "Start Reading" button) in `LandingPage.tsx` (FR-002)
- [x] T011 [US1] Implement animation/hover effect for "Start Reading" button using `LandingPage.animations.ts` (FR-003)
- [x] T012 [US1] Implement side section (Module-chapter listing, icons/placeholders) in `LandingPage.tsx` (FR-005)
- [x] T013 [P] [US1] Create `src/components/Shared/ModuleCard.tsx` for reusable module display
- [x] T014 [US1] Integrate `ModuleCard.tsx` into `LandingPage.tsx`
- [x] T015 [US1] Configure `src/pages/index.tsx` to render the `LandingPage` component (FR-001)
- [x] T016 [US1] Apply modern theme (dark/light contrast, bold fonts) using CSS Modules and Docusaurus theme overrides (FR-006, FR-009)
- [x] T017 [US1] Ensure mobile-friendly responsive layout (FR-007)
- [x] T018 [P] [US1] Add placeholder images/icons to `static/img/` for modules

---

## Phase 3: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T019 Documentation updates (e.g., update `quickstart.md` if necessary)
- [x] T020 Perform visual review and cross-browser compatibility checks (SC-005)
- [ ] T021 Run `quickstart.md` validation (SC-001)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **User Story 1 (Phase 2)**: Depends on Setup completion
- **Polish (Phase 3)**: Depends on User Story 1 being complete

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Component creation before integration
- Core implementation before styling/animations
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All tests for a user story marked [P] can run in parallel
- Styling and animation tasks marked [P] can run in parallel
- Adding placeholder images/icons marked [P] can run in parallel

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: User Story 1
3. **STOP and VALIDATE**: Test User Story 1 independently
4. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup
2. Complete User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Complete Polish tasks → Final review and validation

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
