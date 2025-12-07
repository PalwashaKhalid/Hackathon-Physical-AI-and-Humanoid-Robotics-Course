<!--
Sync Impact Report:
Version change: None (initial) -> 0.1.0
Modified principles:
  - PRINCIPLE_1_NAME -> Technical Accuracy
  - PRINCIPLE_2_NAME -> Clarity for Developers
  - PRINCIPLE_3_NAME -> Modularity
  - PRINCIPLE_4_NAME -> Actionability
  - PRINCIPLE_5_NAME -> Key Standards
Added sections:
  - Constraints
  - Success Criteria
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated
  - .specify/templates/spec-template.md: ✅ updated
  - .specify/templates/tasks-template.md: ✅ updated
  - .specify/templates/commands/*.md: ✅ updated
Follow-up TODOs: None
-->
# AI-Driven Technical Book Built with Docusaurus Constitution

## Core Principles

### Technical Accuracy
Technical accuracy in topics like cloud, AI, vector databases, and RAG workflows.

### Clarity for Developers
Content must be clear and accessible for beginner-to-intermediate software engineers.

### Modularity
Chapters must be structured modularly to fit Docusaurus requirements.

### Actionability
Content must include practical steps, code snippets, and deployable instructions.

### Key Standards
*   All architecture, tools, and APIs must be explained with working examples.
*   Code must follow TypeScript/JS best practices where relevant.
*   Diagrams should be simple, clear, and reproducible.
*   Writing tone: tutorial + reference manual.
*   Use concise explanations, avoid unnecessary theory.
*   Include step-by-step guides for:
    *   Docusaurus setup
    *   GitHub Pages deployment
    *   RAG pipeline using OpenAI Agents/ChatKit + Gemini API
    *   Qdrant + Neon Serverless Postgres integration

## Constraints

*   Book length: 8–12 chapters
*   Each chapter: 1,000–2,000 words
*   Include minimum one code example per concept
*   All instructions must be runnable on Windows + Linux
*   Output format: Docusaurus-compatible Markdown

## Success Criteria

*   Book builds successfully in Docusaurus
*   GitHub Pages deploys without errors
*   RAG chatbot runs end-to-end: Ingestion → Embeddings → Qdrant → Retrieval → OpenAI/Gemini → Response
*   All examples tested and functional
*   Writing meets: Clear, structured, developer-friendly tone, Zero plagiarism

## Governance
All PRs/reviews must verify compliance; Complexity must be justified; Use `.specify/memory/constitution.md` for runtime development guidance. Amendments require documentation, approval, and a migration plan.

**Version**: 0.1.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
