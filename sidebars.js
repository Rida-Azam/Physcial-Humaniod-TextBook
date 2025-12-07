/**
 * Creating a sidebar enables you to:
 - Create an ordered group of docs
 - Render a sidebar for each doc of that group
 - By default, Docusaurus uses a sidebar from the docs folder.
 - If you use multiple docs folders, you can configure them in the docs options.
 */

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Manual sidebar structure for the textbook
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'MODULE 1',
      items: [
        'module1/chapter1',
        'module1/chapter2',
        'module1/chapter3',
        'module1/chapter4',
        'module1/chapter5',
      ],
    },
    {
      type: 'category',
      label: 'MODULE 2',
      items: [
        'module2/chapter6',
        'module2/chapter7',
        'module2/chapter8',
        'module2/chapter9',
      ],
    },
    {
      type: 'category',
      label: 'MODULE 3',
      items: [
        'module3/chapter10',
        'module3/chapter11',
        'module3/chapter12',
      ],
    },
    {
      type: 'category',
      label: 'MODULE 4',
      items: [
        'module4/chapter13',
        'module4/chapter14',
        'module4/chapter15',
      ],
    },
    {
      type: 'category',
      label: 'FINAL BOOK REVIEW',
      items: [
        'final_review/project_summary',
        'final_review/plagiarism_check_report',
        'final_review/fact_check_report',
        'final_review/final_review_process',
      ],
    },
    'citation_guide',
    'review_process',
  ],
};

export default sidebars;
