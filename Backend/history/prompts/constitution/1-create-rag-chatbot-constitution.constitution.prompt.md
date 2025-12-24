---
id: 1
title: "Create RAG Chatbot Constitution"
stage: constitution
date_iso: "2025-01-15"
surface: "agent"
model: "Qwen"
feature: "none"
branch: "main"
user: "User"
command: "/sp.constitution Project: Integrated RAG Chatbot for an AI-Native Interactive Book"
labels: ["constitution", "rag", "chatbot", "ai"]
spec: null
ticket: null
adr: null
pr: null
files_yaml:
  - ".specify/memory/constitution.md"
tests_yaml: []
links: []
---

# Prompt History Record: Create RAG Chatbot Constitution

## Outcome
Successfully created a project constitution for the Integrated RAG Chatbot project with principles and guidelines tailored to the specific requirements.

## Evaluation
The constitution document was created with all required sections including Core Principles, Technical Constraints, Development Standards, Documentation Requirements, and Governance. It properly reflects the specific requirements of the RAG chatbot project with its focus on faithfulness, precision, modularity, professional engineering, privacy & control, and AI-native design.

## Prompt Text
```
$ARGUMENTS
```

## Response Text
Created a comprehensive constitution for the Integrated RAG Chatbot project that includes:

- Core principles focusing on faithfulness, precision, modularity, professional engineering, privacy & control, and AI-native design
- Technical constraints including LLM requirements (Cohere only, no OpenAI), architecture standards (FastAPI, Qdrant, Neon PostgreSQL), and RAG patterns
- Development standards for code quality, testing, and security
- Documentation requirements and deployment expectations
- Governance guidelines to ensure all future development adheres to these principles