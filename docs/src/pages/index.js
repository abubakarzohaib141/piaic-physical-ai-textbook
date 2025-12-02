import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>
            Learn Physical AI in <span className={styles.highlight}>Live Online</span>
            <br />
            <span className={styles.highlight}>Classes</span> by Experts
          </h1>
          <p className={styles.heroSubtitle}>
            Master ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models - Choose Between Dedicated English or Urdu/Hindi Sections!
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--primary button--lg', styles.getStarted)}
              to="/docs/physical-ai/introduction">
              Start Your AI Journey
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function Feature({ icon, title, description }) {
  return (
    <div className={clsx('col col--3', styles.feature)}>
      <div className={styles.featureCard}>
        <div className={styles.featureIcon}>{icon}</div>
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <Feature
            icon="🎮"
            title="Module 1: ROS 2"
            description="Master the robot operating system with nodes, topics, and URDF"
          />
          <Feature
            icon="🌍"
            title="Module 2: Gazebo"
            description="Create digital twins with physics-accurate simulation"
          />
          <Feature
            icon="⚡"
            title="Module 3: Isaac"
            description="GPU-accelerated perception with NVIDIA Isaac platform"
          />
          <Feature
            icon="🗣️"
            title="Module 4: VLA"
            description="Voice commands to robot actions with LLMs"
          />
        </div>
      </div>
    </section>
  );
}

function Stats() {
  return (
    <section className={styles.stats}>
      <div className="container">
        <h2 className={styles.statsTitle}>Course at a Glance</h2>
        <div className={styles.statsGrid}>
          <div className={styles.statCard}>
            <div className={styles.statNumber}>13</div>
            <div className={styles.statLabel}>Weeks</div>
          </div>
          <div className={styles.statCard}>
            <div className={styles.statNumber}>4</div>
            <div className={styles.statLabel}>Modules</div>
          </div>
          <div className={styles.statCard}>
            <div className={styles.statNumber}>12+</div>
            <div className={styles.statLabel}>Projects</div>
          </div>
          <div className={styles.statCard}>
            <div className={styles.statNumber}>100%</div>
            <div className={styles.statLabel}>Practical</div>
          </div>
        </div>
      </div>
    </section>
  );
}

function LearningPath() {
  return (
    <section className={styles.learningPath}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Your Learning Journey</h2>
        <div className={styles.pathContainer}>
          <div className={styles.pathStep}>
            <div className={styles.stepNumber}>1</div>
            <h3>Foundations</h3>
            <p>ROS 2 fundamentals, sensors, and robot description</p>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathStep}>
            <div className={styles.stepNumber}>2</div>
            <h3>Simulation</h3>
            <p>Gazebo physics and sensor simulation</p>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathStep}>
            <div className={styles.stepNumber}>3</div>
            <h3>AI Integration</h3>
            <p>NVIDIA Isaac and GPU acceleration</p>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathStep}>
            <div className={styles.stepNumber}>4</div>
            <h3>Voice Control</h3>
            <p>LLM-powered natural language commands</p>
          </div>
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.cta}>
      <div className="container">
        <div className={styles.ctaContent}>
          <h2>Ready to Build the Future?</h2>
          <p>Join thousands learning to create intelligent humanoid robots</p>
          <Link
            className={clsx('button button--primary button--lg', styles.ctaButton)}
            to="/docs/intro">
            Get Started Now →
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Learn Physical AI and Humanoid Robotics - From ROS 2 to Vision-Language-Action models">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <Stats />
        <LearningPath />
        <CTASection />
      </main>
    </Layout>
  );
}
