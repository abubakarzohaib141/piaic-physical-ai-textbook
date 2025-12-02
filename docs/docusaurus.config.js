const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').DocusaurusConfig} */
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging the gap between digital intelligence and the physical world',
  url: 'https://abubakarzohaib141.github.io', // your GitHub Pages domain
  baseUrl: '/piaic-physical-ai-textbook/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',
  organizationName: 'abubakarzohaib141', // GitHub username
  projectName: 'piaic-physical-ai-textbook', // repo name

  presets: [
    [
      '@docusaurus/preset-classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/abubakarzohaib141/piaic-physical-ai-textbook/edit/main/docs/',
        },
        blog: {
          showReadingTime: true,
          editUrl: 'https://github.com/abubakarzohaib141/piaic-physical-ai-textbook/edit/main/docs/blog/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
  themeConfig: {
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'doc',
          docId: 'physical-ai/introduction',
          position: 'left',
          label: 'Get Started',
        },
        {
          type: 'doc',
          docId: 'physical-ai/introduction',
          position: 'left',
          label: 'Course',
        },
        {
          type: 'dropdown',
          label: 'Modules',
          position: 'left',
          items: [
            { type: 'doc', docId: 'physical-ai/module1-ros2/overview', label: 'Module 1: ROS 2' },
            { type: 'doc', docId: 'physical-ai/module2-gazebo/overview', label: 'Module 2: Gazebo' },
            { type: 'doc', docId: 'physical-ai/module3-isaac/overview', label: 'Module 3: NVIDIA Isaac' },
            { type: 'doc', docId: 'physical-ai/module4-vla/overview', label: 'Module 4: VLA' },
          ],
        },
        {
          type: 'doc',
          docId: 'physical-ai/week-by-week/outline',
          position: 'left',
          label: 'Weekly Schedule',
        },
        { href: 'https://panaversity.org', label: 'Panaversity', position: 'right' },
        { href: 'https://github.com/abubakarzohaib141/piaic-physical-ai-textbook', label: 'GitHub', position: 'right' },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learn',
          items: [
            { label: 'Get Started', to: '/docs/physical-ai/introduction' },
            { label: 'Course Introduction', to: '/docs/physical-ai/introduction' },
            { label: 'Weekly Schedule', to: '/docs/physical-ai/week-by-week/outline' },
          ],
        },
        {
          title: 'Modules',
          items: [
            { label: 'ROS 2', to: '/docs/physical-ai/module1-ros2/overview' },
            { label: 'Gazebo & Unity', to: '/docs/physical-ai/module2-gazebo/overview' },
            { label: 'NVIDIA Isaac', to: '/docs/physical-ai/module3-isaac/overview' },
            { label: 'Vision-Language-Action', to: '/docs/physical-ai/module4-vla/overview' },
          ],
        },
        {
          title: 'Community',
          items: [
            { label: 'Panaversity', href: 'https://panaversity.org' },
            { label: 'PIAIC', href: 'https://piaic.org' },
            { label: 'GIAIC', href: 'https://giaic.org' },
          ],
        },
        {
          title: 'More',
          items: [
            { label: 'GitHub', href: 'https://github.com/abubakarzohaib141/piaic-physical-ai-textbook' },
            { label: 'AI-Native Book', href: 'https://ai-native.panaversity.org' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Panaversity. Built with Docusaurus.`,
    },
    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
    },
    colorMode: { defaultMode: 'light', disableSwitch: false },
    hideableSidebar: true,
  },
};
