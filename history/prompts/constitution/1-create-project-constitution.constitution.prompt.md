---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: gemini-pro
feature: none
branch: main
user: User
command: /sp.constitution
labels: [constitution, setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
tests:
  - none
---

## Prompt

/sp.constitution Step 1: Run /sp.constitutionOpen your AI tool and run the constitution command with your project requirements:Project: AI-Spec–Driven Technical Book on Physical AI & Humanoid Robotics with Integrated RAG ChatbotCore principles:- Technical accuracy from primary sources- Clarity for AI/Robotics students and developers- Reproducibility of all code and simulations- Engineering rigor and Spec-Kit Plus alignmentKey standards:- All claims must be source-traceable- Citation format: APA- Sources: ≥60% peer-reviewed or official vendor docs- Plagiarism: 0% tolerance- Writing level: Flesch-Kincaid 11–13- All code must be executable and validatedConstraints:- Format: Docusaurus book deployed to GitHub Pages- Minimum 12 chapters, 25 references- Length: 35k–60k words- Robotics stack: ROS 2, Gazebo, Unity, NVIDIA Isaac Sim & Isaac ROS- RAG stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud- RAG must answer from book and user-selected text onlySuccess criteria:- Successful GitHub Pages deployment- Verified robot simulations and ROS pipelines- Accurate, grounded RAG responses- Zero plagiarism with full reproducibility

## Response snapshot

I have created the project constitution and updated the relevant templates.

## Outcome

- ✅ Impact: Created the project constitution and updated dependent templates.
- 🧪 Tests: No tests were run.
- 📁 Files:
  - .specify/memory/constitution.md (created)
  - .specify/templates/plan-template.md (modified)
- 🔁 Next prompts: none
- 🧠 Reflection: The process of creating the constitution and updating the templates was successful.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
