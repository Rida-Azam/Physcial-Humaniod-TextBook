// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1 - The Robotic Nervous System (ROS 2)',
      items: [
        'module1/chapter1',
        'module1/chapter2',
        'module1/chapter3',
        'module1/chapter4',
        'module1/chapter5'
      ],
    },
    {
      type: 'category',
      label: 'Module 2 - The Digital Twin (Simulation)',
      items: [
        'module2/chapter6',
        'module2/chapter7',
        'module2/chapter8',
        'module2/chapter9'
      ],
    },
    {
      type: 'category',
      label: 'Module 3 - The AI-Robot Brain (NVIDIA Isaac Platform)',
      items: [
        'module3/chapter10',
        'module3/chapter11',
        'module3/chapter12'
      ],
    },
    {
      type: 'category',
      label: 'Module 4 - Vision-Language-Action & Conversational Humanoids',
      items: [
        'module4/chapter13',
        'module4/chapter14',
        'module4/chapter15'
      ],
    },
  ],
};

module.exports = sidebars;