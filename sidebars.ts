import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar for the Physical AI & Humanoid Robotics book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics',
      collapsed: false,
      items: [
        'chatbot-integration',
        'reusable-intelligence-system',
        'authentication-personalization',
        'intro',
        'chapter1-fundamentals',
        'chapter2-ros2',
        'chapter3-simulation',
        'chapter4-isaac',
        'chapter5-vla-systems',
        'chapter6-complete-system',
        'chapter7-advanced-topics'
      ]
    }
  ],
};

export default sidebars;
