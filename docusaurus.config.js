import { themes as prismThemes } from 'prism-react-renderer';
import path from 'path';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Practical, Modern Textbook for Building Embodied Intelligent Systems Using ROS 2, Gazebo, Isaac Sim, and Vision-Language-Action Pipelines.',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-github-username.github.io',
  // Set the /<baseUrl>/ path under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/physical-ai-textbook/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-github-username', // Usually your GitHub org/user name.
  projectName: 'physical-ai-textbook', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang.
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  plugins: [
    path.resolve(__dirname, './plugins/thebe-plugin'),
  ],

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo. For example: aloisdeniel/docusaurus-plugin-armada
          // Remove this to disable the docs plugin.
          editUrl:
            'https://github.com/your-github-username/physical-ai-textbook/tree/main/',
        },
        blog: false, // Disabled blog as requested
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig: {
    thebe: {
      userName: 'thebe_user',
      repository: 'your-github-username/physical-ai-textbook',
      branch: 'main',
      selector: 'div.highlight code',
      kernelOptions: {
        name: 'python',
        language: 'python',
      },
      binderOptions: {
        repo: 'your-github-username/physical-ai-textbook',
        binderurl: 'https://mybinder.org',
        branch: 'main',
        filepath: ''
      },
      local: false,
    },
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/your-github-username/physical-ai-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
      hideOnScroll: true, // Mobile-friendly navbar
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'About',
          items: [
            {
              label: 'Physical AI & Humanoid Robotics',
              to: '/',
            },
            {
              label: 'A Complete Textbook for Building Embodied Intelligent Systems',
              to: '/',
            },
          ],
        },
        {
          title: 'Quick Links',
          items: [
            {
              label: 'Start Reading',
              to: '/docs/intro',
            },
            {
              label: 'Module Overview',
              to: '/',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/your-github-username/physical-ai-textbook',
            },
            {
              label: 'Robotics Stack Exchange',
              href: 'https://robotics.stackexchange.com/',
            },
            {
              label: 'ROS Answers',
              href: 'https://answers.ros.org/',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. All rights reserved.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
    // Enhanced accessibility and mobile features
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 5,
    },
    docs: {
      sidebar: {
        hideable: true,
        autoCollapseCategories: true,
      },
    },
  },
};

export default config;
