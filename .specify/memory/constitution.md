<!--
Sync Impact Report:
- Version change: 0.0.0 → 1.0.0
- List of modified principles:
  - [PRINCIPLE_1_NAME] → Technical Accuracy
  - [PRINCIPLE_2_NAME] → Clarity
  - [PRINCIPLE_3_NAME] → Reproducibility
  - [PRINCIPLE_4_NAME] → Engineering Rigor
  - [PRINCIPLE_5_NAME] → Source Traceability
  - [PRINCIPLE_6_NAME] → Plagiarism-Free
- Added sections: Key Standards, Constraints, Success Criteria
- Removed sections: None
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->
# AI-Spec–Driven Technical Book on Physical AI & Humanoid Robotics with Integrated RAG Chatbot Constitution

## Core Principles

### I. Technical Accuracy
All claims, explanations, and technical descriptions must be derived from and traceable to primary sources, such as peer-reviewed research or official vendor documentation.

### II. Clarity
The content must be written for clarity, targeting an audience of AI/Robotics students and developers. Writing level should adhere to a Flesch-Kincaid grade level of 11–13.

### III. Reproducibility
All code examples, simulations, and datasets must be fully reproducible by readers. This includes providing all necessary configurations, scripts, and environment details.

### IV. Engineering Rigor
The project will adhere to the principles of Spec-Kit Plus, ensuring a systematic and spec-driven development process for all software and content.

### V. Source Traceability
All factual claims and technical assertions must be traceable to their original source. Citations must follow the APA format. A minimum of 60% of sources must be from peer-reviewed academic papers or official vendor documentation.

### VI. Plagiarism-Free
A zero-tolerance policy for plagiarism is in effect. All content must be original or properly attributed.

## Key Standards

- **Source Traceability:** All claims must be source-traceable.
- **Citation Format:** APA style is mandatory for all citations.
- **Source Quality:** At least 60% of references must be from peer-reviewed journals or official vendor documentation.
- **Plagiarism:** 0% tolerance. Automated checks will be enforced.
- **Writing Level:** Content must target a Flesch-Kincaid grade level of 11–13.
- **Code Validation:** All code must be executable and validated to produce the expected results.

## Constraints

- **Format:** The final output will be a Docusaurus book, deployed to GitHub Pages.
- **Content Minimums:** The book must contain a minimum of 12 chapters and 25 unique references.
- **Length:** The total word count shall be between 35,000 and 60,000 words.
- **Robotics Stack:** The primary robotics stack is ROS 2, Gazebo, Unity, NVIDIA Isaac Sim, and Isaac ROS.
- **RAG Stack:** The RAG chatbot will be built using OpenAI Agents/ChatKit, FastAPI, Neon Postgres, and Qdrant Cloud.
- **RAG Scope:** The RAG chatbot must only answer questions based on the book's content and user-selected supplementary texts.

## Success Criteria

- **Deployment:** The Docusaurus book is successfully built and deployed to a public GitHub Pages site.
- **Simulations:** All robot simulations and ROS pipelines are verified and run as described.
- **RAG Accuracy:** The RAG chatbot provides accurate, grounded responses based on the specified knowledge base.
- **Reproducibility:** A third party can successfully reproduce all code examples and simulations.

## Governance

This Constitution is the single source of truth for project principles and standards. All project activities, contributions, and reviews must align with it. Amendments to this constitution require a formal proposal, review, and approval process.

- **Compliance:** All pull requests and code reviews must verify compliance with this constitution.
- **Versioning:** Changes to this document will follow semantic versioning (MAJOR.MINOR.PATCH).
  - **MAJOR:** Backward-incompatible changes or removal of a core principle.
  - **MINOR:** Addition of a new principle or significant expansion of a section.
  - **PATCH:** Minor clarifications, typo fixes, or rephrasing.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07