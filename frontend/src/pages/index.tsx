import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading Free →
          </Link>
          <Link
            className="button button--outline button--lg"
            to="https://www.youtube.com/watch?v=example_demo">
            Watch Capstone Demo
          </Link>
        </div>
      </div>
    </header>
  );
}

// Supportive Section Component
function SupportiveSection() {
  return (
    <section className={styles.supportiveSection}>
      <div className="container">
        <div className="row">
          <div className="col col--6">
            
          </div>
          <div className="col col--6">
            <h2>Learn to Build Autonomous Humanoids in 2025</h2>
            <p>
              Master ROS 2, NVIDIA Isaac Sim, Vision-Language-Action (VLA) models,
              and deploy voice-controlled humanoids that clean rooms, fold laundry,
              and navigate real-world environments — all in simulation first, then real hardware.
            </p>
            <ul>
              <li>12 Industry-Standard Modules</li>
              <li>Full Capstone: Voice → Plan → Act Humanoid</li>
              <li>Urdu + Roman Urdu Support</li>
              <li>Live Code + Interactive Quizzes</li>
              <li>RAG Chatbot with Selected-Text Explain</li>
            </ul>
          </div>
        </div>
      </div>
    </section>
  );
}

// Social Proof Section Component
function SocialProofSection() {
  const logos = [
    { name: "Panaversity", src: "/img/logos/panaversity.png" },
    { name: "PIAIC", src: "/img/logos/piaic.png" },
    { name: "NVIDIA Isaac", src: "/img/logos/nvidia-isaac.png" },
    { name: "Tesla Optimus", src: "/img/logos/tesla-optimus.png" },
    { name: "Boston Dynamics", src: "/img/logos/bd.png" },
    { name: "Figure AI", src: "/img/logos/figure.png" },
  ];

  return (
    <section className={styles.socialProofSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Trusted by the Leaders in Robotics & AI</h2>
        <div className={styles.logosContainer}>
          {logos.map((logo, index) => (
            <div key={index} className={styles.logoItem}>
              <img src={logo.src} alt={logo.name} title={logo.name} />
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Tools Grid Section Component
function ToolsGridSection() {
  const tools = [
    { name: "ROS 2 Humble", icon: "ros" },
    { name: "NVIDIA Isaac Sim", icon: "nvidia" },
    { name: "GPT-4o Realtime API", icon: "openai" },
    { name: "OpenAI ChatKit", icon: "chatkit" },
    { name: "Qdrant Vector DB", icon: "qdrant" },
    { name: "Gazebo Ignition", icon: "gazebo" },
    { name: "Unity Robotics", icon: "unity" },
    { name: "Claude Subagents", icon: "anthropic" },
  ];

  return (
    <section className={styles.toolsSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Built with the Exact 2025 Stack</h2>
        <div className={styles.toolsGrid}>
          {tools.map((tool, index) => (
            <div key={index} className={styles.toolItem}>
              <div className={styles.toolIcon}>
                <span role="img" aria-label={tool.icon}>{getIconForTool(tool.icon)}</span>
              </div>
              <div className={styles.toolName}>{tool.name}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Helper function to get icons for tools
function getIconForTool(iconName) {
  const icons = {
    ros: "🤖",
    nvidia: "🎮",
    openai: "🧠",
    chatkit: "💬",
    qdrant: "🔍",
    gazebo: "🎬",
    unity: "🧩",
    anthropic: "🧠"
  };
  return icons[iconName] || "🛠️";
}

// Feature Cards Section Component
function FeatureCardsSection() {
  const cards = [
    {
      title: "Capstone Project",
      description: "Build a full VLA humanoid that responds to voice commands",
      icon: "🤖"
    },
    {
      title: "Urdu + Roman Urdu",
      description: "Full translation toggle – first in Pakistan",
      icon: "🌐"
    },
    {
      title: "Live Coding",
      description: "Edit & run ROS 2 + Python code directly in browser",
      icon: "💻"
    },
    {
      title: "RAG Chatbot",
      description: "Select any text → click Explain → perfect answer",
      icon: "💡"
    }
  ];

  return (
    <section className={styles.featureCardsSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Why This Course is Different</h2>
        <div className="row">
          {cards.map((card, index) => (
            <div key={index} className="col col--3">
              <div className={styles.featureCard}>
                <div className={styles.cardIcon}>{card.icon}</div>
                <h3>{card.title}</h3>
                <p>{card.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Final CTA Section Component
function FinalCtaSection() {
  return (
    <section className={styles.finalCtaSection}>
      <div className="container">
        <h2>Ready to Build the Future?</h2>
        <p>
          Join thousands of students learning the exact skills needed to build and deploy autonomous humanoid robots
        </p>
        <Link
          className="button button--primary button--lg"
          to="/docs/intro">
          Start Learning Now – 100% Free
        </Link>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook - Embodied Intelligence in the Real World">
      <HomepageHeader />
      <main>
        <SupportiveSection />
        <SocialProofSection />
        <ToolsGridSection />
        <FeatureCardsSection />
        <FinalCtaSection />
      </main>
    </Layout>
  );
}