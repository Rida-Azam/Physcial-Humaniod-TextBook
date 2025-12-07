import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-textbook/docs',
    component: ComponentCreator('/physical-ai-textbook/docs', '688'),
    routes: [
      {
        path: '/physical-ai-textbook/docs',
        component: ComponentCreator('/physical-ai-textbook/docs', '5ba'),
        routes: [
          {
            path: '/physical-ai-textbook/docs',
            component: ComponentCreator('/physical-ai-textbook/docs', '317'),
            routes: [
              {
                path: '/physical-ai-textbook/docs/citation_guide',
                component: ComponentCreator('/physical-ai-textbook/docs/citation_guide', '80e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/final_review/fact_check_report',
                component: ComponentCreator('/physical-ai-textbook/docs/final_review/fact_check_report', 'f8f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/final_review/final_review_process',
                component: ComponentCreator('/physical-ai-textbook/docs/final_review/final_review_process', 'd8a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/final_review/plagiarism_check_report',
                component: ComponentCreator('/physical-ai-textbook/docs/final_review/plagiarism_check_report', '405'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/final_review/project_summary',
                component: ComponentCreator('/physical-ai-textbook/docs/final_review/project_summary', '23d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/intro',
                component: ComponentCreator('/physical-ai-textbook/docs/intro', '2bc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module1/chapter1',
                component: ComponentCreator('/physical-ai-textbook/docs/module1/chapter1', '707'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module1/chapter2',
                component: ComponentCreator('/physical-ai-textbook/docs/module1/chapter2', '9be'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module1/chapter3',
                component: ComponentCreator('/physical-ai-textbook/docs/module1/chapter3', 'df0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module1/chapter4',
                component: ComponentCreator('/physical-ai-textbook/docs/module1/chapter4', 'c49'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module1/chapter5',
                component: ComponentCreator('/physical-ai-textbook/docs/module1/chapter5', '88b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module2/chapter10',
                component: ComponentCreator('/physical-ai-textbook/docs/module2/chapter10', 'df1'),
                exact: true
              },
              {
                path: '/physical-ai-textbook/docs/module2/chapter6',
                component: ComponentCreator('/physical-ai-textbook/docs/module2/chapter6', 'eca'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module2/chapter7',
                component: ComponentCreator('/physical-ai-textbook/docs/module2/chapter7', 'aeb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module2/chapter8',
                component: ComponentCreator('/physical-ai-textbook/docs/module2/chapter8', '453'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module2/chapter9',
                component: ComponentCreator('/physical-ai-textbook/docs/module2/chapter9', '561'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module3/chapter10',
                component: ComponentCreator('/physical-ai-textbook/docs/module3/chapter10', '6de'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module3/chapter11',
                component: ComponentCreator('/physical-ai-textbook/docs/module3/chapter11', '42d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module3/chapter12',
                component: ComponentCreator('/physical-ai-textbook/docs/module3/chapter12', '5b0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module4/chapter13',
                component: ComponentCreator('/physical-ai-textbook/docs/module4/chapter13', '4a5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module4/chapter14',
                component: ComponentCreator('/physical-ai-textbook/docs/module4/chapter14', '650'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module4/chapter15',
                component: ComponentCreator('/physical-ai-textbook/docs/module4/chapter15', 'fa4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/review_process',
                component: ComponentCreator('/physical-ai-textbook/docs/review_process', '24b'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/physical-ai-textbook/',
    component: ComponentCreator('/physical-ai-textbook/', '94f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
