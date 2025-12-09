# Citation Workflow Guide

This guide outlines the recommended citation workflow for the Physical AI & Humanoid Robotics textbook, ensuring consistent APA 7th edition formatting and seamless integration with MDX content.

## 1. Zotero for Reference Management

Zotero is the primary tool for collecting, organizing, and managing all research references. It allows for easy import of citations from various sources (web pages, academic databases, PDFs).

-   **Installation**: Download and install Zotero from [zotero.org](https://www.zotero.org/download/).
-   **Connector**: Install the Zotero Browser Connector for your preferred browser (Chrome, Firefox, Edge) to quickly save references.
-   **Collection**: Create a dedicated collection within Zotero for this textbook project to keep references organized.

## 2. BetterBibTeX for Zotero Integration

BetterBibTeX is a Zotero extension that provides advanced BibTeX export capabilities, crucial for converting Zotero collections into a format compatible with our MDX workflow.

-   **Installation**: Install BetterBibTeX from its GitHub releases page ([retorquere/zotero-better-bibtex](https://github.com/retorquere/zotero-better-bibtex/releases)). In Zotero, go to `Tools > Add-ons`, then drag and drop the `.xpi` file onto the Add-ons window.
-   **Configuration**: Configure BetterBibTeX to export in a suitable format (e.g., `BibLaTeX` or `BibTeX` with `extended` options for full data export).
-   **Export**: Right-click on your Zotero collection and select `Export Collection...`. Choose `Better BibLaTeX` or `Better BibTeX` as the format and save the `.bib` file (e.g., `references.bib`) in your project's `src/references/` directory.

## 3. APA 7th Edition Conversion for MDX

To integrate citations into MDX files and ensure APA 7th edition style, we will use a tool or script to process the `.bib` file and generate MDX-compatible citation keys and formatted bibliographies.

-   **Intermediate Step (Manual/Scripted)**: A custom script (e.g., a Python script using `bibtexparser` and `pandoc` or `citeproc-js`) will be developed to:
    1.  Read the `references.bib` file.
    2.  Generate short citation keys (e.g., `[@AuthorYear]`) for in-text citations in MDX.
    3.  Generate a formatted bibliography in APA 7th edition style (e.g., a markdown or JSON output).
-   **MDX Integration**:
    -   In your `.mdx` chapter files, use the generated short citation keys for in-text citations (e.g., `As discussed by Smith (2023) [@Smith2023], physical AI...`).
    -   Include the generated bibliography at the end of each chapter or module, as specified in the `docusaurus.config.js` or through a custom MDX component.

## Workflow Summary

1.  **Collect** references in Zotero.
2.  **Export** Zotero collection to `.bib` using BetterBibTeX.
3.  **Process** `.bib` file with custom script to generate MDX-compatible citations and bibliography.
4.  **Integrate** citations and bibliography into `.mdx` chapter files.

This systematic approach ensures accurate, consistent, and easily manageable citations throughout the textbook content.
