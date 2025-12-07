# Research: Docusaurus Site Architecture

## Theme/Plugins

**Decision**: Use the default Docusaurus theme with the following plugins:
- `@docusaurus/plugin-content-docs`
- `@docusaurus/plugin-content-blog`
- `@docusaurus/plugin-content-pages`
- `@docusaurus/plugin-sitemap`

**Rationale**: The default theme is clean, well-documented, and provides a good starting point. The selected plugins are essential for creating a documentation site with docs, a blog, and static pages.

**Alternatives considered**: Custom theme, but this would add unnecessary complexity at this stage.

## Module vs Chapter Layout

**Decision**: Each module will be a top-level category in the docs sidebar. Each chapter will be a document within that category.

**Rationale**: This provides a clear and organized structure for the content.

**Alternatives considered**: A flat structure, but this would be less organized for a multi-module book.

## RAG Chatbot Embed Location

**Decision**: The RAG chatbot will be embedded as a floating component on all pages of the Docusaurus site.

**Rationale**: This makes the chatbot easily accessible from anywhere in the book.

**Alternatives considered**: A dedicated page for the chatbot, but this would be less convenient for users who want to ask questions while reading.
