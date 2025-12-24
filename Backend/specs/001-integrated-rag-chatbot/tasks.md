---

description: "Task list for Integrated RAG Chatbot feature"
---

# Tasks: Integrated RAG Chatbot

**Input**: Design documents from `/specs/001-integrated-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Unit tests for retrieval logic, context filtering, and selected-text mode are included as per requirements.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- Paths shown below follow plan.md structure

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project structure per implementation plan in src/
- [X] T002 Initialize Python project with FastAPI, Cohere, Qdrant, Pydantic, asyncpg dependencies
- [X] T003 [P] Configure linting and formatting tools (ruff, black, mypy)
- [X] T004 Create requirements.txt with all required dependencies
- [X] T005 Create .env configuration file with appropriate environment variables

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [X] T006 Setup configuration loading from environment variables in src/utils/config_loader.py
- [X] T007 [P] Create base models/entities that all stories depend on
- [X] T008 Configure error handling and logging infrastructure in src/utils/logging_utils.py
- [X] T009 Setup environment configuration management
- [X] T010 Implement utility functions for token management in src/utils/token_utils.py
- [X] T011 Setup Cohere API client integration
- [X] T012 Setup Qdrant client connection and configuration

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Full-book question answering (Priority: P1) 🎯 MVP

**Goal**: Enable readers to ask questions about the book content and receive accurate answers based only on the book's content with proper chapter/section references.

**Independent Test**: Reader can ask a specific question about the book content and receive a response that is grounded only in the book's content with appropriate chapter/section references.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T013 [P] [US1] Contract test for query endpoint in tests/contract/test_query.py
- [X] T014 [P] [US1] Unit test for retrieval service in tests/unit/test_retrieval_generation_services.py
- [X] T015 [P] [US1] Unit test for generation service in tests/unit/test_retrieval_generation_services.py

### Implementation for User Story 1

- [X] T016 [P] [US1] Create BookContent model in src/models/book_content.py
- [X] T017 [P] [US1] Create RetrievedChunk model in src/models/retrieved_chunk.py
- [X] T018 [P] [US1] Create UserQuery model in src/models/user_query.py
- [X] T019 [P] [US1] Create Response model in src/models/response.py
- [X] T020 [P] [US1] Create SessionContext model in src/models/session_context.py
- [X] T021 [US1] Implement IngestionService in src/services/ingestion_service.py
- [X] T022 [US1] Implement RetrievalService in src/services/retrieval_service.py
- [X] T023 [US1] Implement GenerationService in src/services/generation_service.py
- [X] T024 [US1] Implement ContextFilterService in src/services/context_filter_service.py
- [X] T025 [US1] Create query schemas in src/api/schemas/query_schemas.py
- [X] T026 [US1] Create response schemas in src/api/schemas/response_schemas.py
- [X] T027 [US1] Implement query endpoint in src/api/routers/query_router.py
- [X] T028 [US1] Implement ingestion endpoint in src/api/routers/ingestion_router.py
- [X] T029 [US1] Add validation and error handling to query endpoint
- [X] T030 [US1] Add logging for user story 1 operations
- [X] T031 [US1] Implement semantic chunking utility in src/utils/chunking_utils.py

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Selected-text-only question answering (Priority: P2)

**Goal**: Enable readers to select specific text from the book and ask questions only about that selected text, without the system pulling from other parts of the book.

**Independent Test**: Reader can select text from the book content, ask a question about that text, and receive a response that only considers the selected text.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [X] T032 [P] [US2] Contract test for query endpoint with selected_text in tests/contract/test_query_selected_text.py
- [X] T033 [P] [US2] Unit test for context isolation in tests/unit/test_context_isolation.py

### Implementation for User Story 2

- [X] T034 [P] [US2] Enhance UserQuery model to better handle selected text in src/models/user_query.py
- [X] T035 [US2] Modify RetrievalService to support selected-text-only mode in src/services/retrieval_service.py
- [X] T036 [US2] Implement temporary context creation from selected text in src/services/context_filter_service.py
- [X] T037 [US2] Update query endpoint to detect and activate selected-text-only mode in src/api/routers/query_router.py
- [X] T038 [US2] Ensure responses respect strict boundaries in selected-text-only mode
- [X] T039 [US2] Add logging for selected-text-only mode operations

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Chapter/section-aware responses (Priority: P3)

**Goal**: Provide responses that are aware of the chapter/section context, providing relevant information based on the specific part of the book the user is interacting with.

**Independent Test**: When the user is reading in a specific chapter/section, the system prioritizes responses that are relevant to that context.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [X] T040 [P] [US3] Integration test for context-aware responses in tests/integration/test_context_aware.py

### Implementation for User Story 3

- [X] T041 [P] [US3] Modify retrieval service to prioritize current context in src/services/retrieval_service.py
- [X] T042 [US3] Enhance response generation to consider chapter/section context in src/services/generation_service.py
- [X] T043 [US3] Update SessionContext model to manage current context in src/models/session_context.py
- [X] T044 [US3] Update query endpoint to handle context information in src/api/routers/query_router.py

**Checkpoint**: All user stories should now be independently functional

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T045 [P] Documentation updates in docs/
- [X] T046 Code cleanup and refactoring
- [X] T047 Performance optimization across all stories
- [X] T048 [P] Additional unit tests (if requested) in tests/unit/
- [X] T049 Security hardening
- [X] T050 Run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for query endpoint in tests/contract/test_query.py"
Task: "Unit test for retrieval service in tests/unit/test_retrieval_service.py"
Task: "Unit test for generation service in tests/unit/test_generation_service.py"

# Launch all models for User Story 1 together:
Task: "Create BookContent model in src/models/book_content.py"
Task: "Create RetrievedChunk model in src/models/retrieved_chunk.py"
Task: "Create UserQuery model in src/models/user_query.py"
Task: "Create Response model in src/models/response.py"
Task: "Create SessionContext model in src/models/session_context.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence