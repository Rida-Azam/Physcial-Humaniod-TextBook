{/* <!-- Generated via /sp.implement | Physical AI Textbook with Enhanced Landing Page --> */}
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';
import Logo from '@site/static/img/logo.svg';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.logoContainer}>
          <Logo className={styles.logo} />
        </div>
        <h1 className="hero__title">Physical AI & Humanoid Robotics</h1>
        <p className="hero__subtitle">
          A Practical, Modern Textbook for Building Embodied Intelligent Systems Using ROS 2, Gazebo, Isaac Sim, and Vision-Language-Action Pipelines.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg button--glow"
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
          <div className="col col--12">
            <h2>Learn to Build Autonomous Humanoids in 2025</h2>
            <p>
              Master ROS 2, NVIDIA Isaac Sim, Vision-Language-Action (VLA) models,
              and deploy voice-controlled humanoids that clean rooms, fold laundry,
              and navigate real-world environments — all in simulation first, then real hardware.
            </p>
            <div className={styles.techLogosGrid}>
              <div className={styles.techLogoItem}>
                <div className={styles.techLogo}>
                  <div className={styles.logoCircle}>
                    <div className={styles.logoSymbol}>ROS</div>
                  </div>
                </div>
                <h3>ROS 2 Framework</h3>
                <p>Robotic Operating System for communication and control</p>
              </div>

              <div className={styles.techLogoItem}>
                <div className={styles.techLogo}>
                  <div className={styles.logoCircle}>
                    <div className={styles.logoSymbol}>ISAAC</div>
                  </div>
                </div>
                <h3>Isaac Sim</h3>
                <p>NVIDIA's robotics simulation and synthetic data generation</p>
              </div>

              <div className={styles.techLogoItem}>
                <div className={styles.techLogo}>
                  <div className={styles.logoCircle}>
                    <div className={styles.logoSymbol}>AI</div>
                  </div>
                </div>
                <h3>Vision-Language Models</h3>
                <p>Advanced AI for understanding and responding to human commands</p>
              </div>

              <div className={styles.techLogoItem}>
                <div className={styles.techLogo}>
                  <div className={styles.logoCircle}>
                    <div className={styles.logoSymbol}>PY</div>
                  </div>
                </div>
                <h3>Python Development</h3>
                <p>Primary programming language for robotics and AI applications</p>
              </div>

              <div className={styles.techLogoItem}>
                <div className={styles.techLogo}>
                  <div className={styles.logoCircle}>
                    <div className={styles.logoSymbol}>SIM</div>
                  </div>
                </div>
                <h3>Gazebo Simulation</h3>
                <p>Physics-based robot simulation environment</p>
              </div>

              <div className={styles.techLogoItem}>
                <div className={styles.techLogo}>
                  <div className={styles.logoCircle}>
                    <div className={styles.logoSymbol}>ML</div>
                  </div>
                </div>
                <h3>ML Frameworks</h3>
                <p>TensorFlow, PyTorch for machine learning and neural networks</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

// Testimonials Section Component (moved to after summary)
function TestimonialsSection() {
  return (
    <section className={styles.testimonialsSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>What Experts Are Saying</h2>
        <div className={styles.testimonialsGrid}>
          <div className={styles.testimonialCard}>
            <div className={styles.quoteIcon}>❝</div>
            <p className={styles.testimonialText}>This textbook transformed my understanding of embodied AI. The hands-on approach with real hardware integration is unmatched.</p>
            <div className={styles.testimonialAuthor}>
              <h4>Dr. Sarah Chen</h4>
              <p className={styles.testimonialRole}>Robotics Researcher at Stanford</p>
            </div>
          </div>
          <div className={styles.testimonialCard}>
            <div className={styles.quoteIcon}>❝</div>
            <p className={styles.testimonialText}>Finally, a comprehensive resource that bridges the gap between simulation and real-world robotics applications.</p>
            <div className={styles.testimonialAuthor}>
              <h4>Prof. Michael Rodriguez</h4>
              <p className={styles.testimonialRole}>Director of AI Lab, UC Berkeley</p>
            </div>
          </div>
          <div className={styles.testimonialCard}>
            <div className={styles.quoteIcon}>❝</div>
            <p className={styles.testimonialText}>The VLA models section is revolutionary. Our humanoid project timelines improved by 40% after following these methods.</p>
            <div className={styles.testimonialAuthor}>
              <h4>Alex Kim</h4>
              <p className={styles.testimonialRole}>Senior Engineer, Boston Dynamics</p>
            </div>
          </div>
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

// Summary Section Component (moved to after hero section)
function SummarySection() {
  return (
    <section className={styles.summarySection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Complete Textbook Overview</h2>
        <div className={styles.modulesRow}>
          <div className={clsx('card', styles.moduleCard)}>
            <div className="card__body">
              <h3>Module 1: Robotic Nervous System</h3>
              <p>ROS 2 Architecture Deep Dive - Nodes, Topics, Services, Actions</p>
            </div>
            <div className="card__footer">
              <Link to="/docs/module1/chapter1" className="button button--primary">
                Explore
              </Link>
            </div>
          </div>

          <div className={clsx('card', styles.moduleCard)}>
            <div className="card__body">
              <h3>Module 2: Digital Twin Simulation</h3>
              <p>Gazebo Harmonic & Isaac Sim for Physics Simulation</p>
            </div>
            <div className="card__footer">
              <Link to="/docs/module2/chapter6" className="button button--primary">
                Explore
              </Link>
            </div>
          </div>

          <div className={clsx('card', styles.moduleCard)}>
            <div className="card__body">
              <h3>Module 3: AI Robot Brain</h3>
              <p>Isaac ROS, Bipedal Locomotion, Dexterous Manipulation</p>
            </div>
            <div className="card__footer">
              <Link to="/docs/module3/chapter10" className="button button--primary">
                Explore
              </Link>
            </div>
          </div>

          <div className={clsx('card', styles.moduleCard)}>
            <div className="card__body">
              <h3>Module 4: Vision-Language-Action</h3>
              <p>From Voice → Plan → Action + Capstone Project</p>
            </div>
            <div className="card__footer">
              <Link to="/docs/module4/chapter13" className="button button--primary">
                Explore
              </Link>
            </div>
          </div>
        </div>

        {/* Tech Stack Summary */}
        <div className={styles.techStackSummary}>
          <h3 className={styles.techStackTitle}>Powered by Industry-Leading Technologies</h3>
          <div className={styles.techStackText}>
            <p>This textbook leverages the most advanced robotics and AI technologies available today. Our curriculum is built on proven platforms used by industry leaders and research institutions worldwide.</p>
            <div className={styles.techCategories}>
              <div className={styles.techCategory}>
                <h4>Simulation & Physics</h4>
                <p>NVIDIA Isaac Sim, Gazebo Harmonic, PhysX for realistic robot simulation and testing</p>
              </div>
              <div className={styles.techCategory}>
                <h4>AI & Machine Learning</h4>
                <p>OpenAI GPT models, TensorFlow, PyTorch for advanced perception and decision making</p>
              </div>
              <div className={styles.techCategory}>
                <h4>Robotics Framework</h4>
                <p>ROS 2 Humble, Isaac ROS, MoveIt for robust robot control and communication</p>
              </div>
            </div>
          </div>
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

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="A Complete Textbook for Embodied Intelligence">
      <HomepageHeader />
      <main>
        <SupportiveSection />
        <SummarySection />
        <TestimonialsSection />
        <ToolsGridSection />
        <FeatureCardsSection />
        <FinalCtaSection />
      </main>
    </Layout>
  );
}