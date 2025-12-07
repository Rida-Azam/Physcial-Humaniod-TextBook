{/* <!-- Generated via /sp.implement | Physical AI Textbook --> */}
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
            Start Reading
          </Link>
        </div>
      </div>
    </header>
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
        <section className={styles.modules}>
          <div className="container">
            <h2 className={styles.sectionTitle}>Textbook Modules</h2>
            <div className={styles.modulesGrid}>
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
          </div>
        </section>
      </main>
    </Layout>
  );
}