// src/components/LandingPage/LandingPage.tsx
import React from 'react';
import Link from '@docusaurus/Link';
import styles from './LandingPage.module.css'; // Assuming CSS Modules
import ModuleCard from '../Shared/ModuleCard'; // Import ModuleCard

const LandingPage = () => {
  const modules = [
    { name: 'Module 1: Introduction', chapters: ['Chapter 1.1', 'Chapter 1.2'], icon: '/img/module-placeholder-1.png' },
    { name: 'Module 2: Advanced Topics', chapters: ['Chapter 2.1', 'Chapter 2.2'], icon: '/img/module-placeholder-2.png' },
    { name: 'Module 3: Case Studies', chapters: ['Chapter 3.1', 'Chapter 3.2'], icon: '/img/module-placeholder-3.png' },
    { name: 'Module 4: Conclusion', chapters: ['Chapter 4.1', 'Chapter 4.2'], icon: '/img/module-placeholder-4.png' },
  ];

  return (
    <div className={styles.landingPageContainer}>
      <div className={styles.centerSection}>
        <h1 className={styles.bookTitle}>AI-Spec–Driven Technical Book</h1>
        <p className={styles.bookDefinition}>Physical AI & Humanoid Robotics with Integrated RAG Chatbot</p>
        <Link to="/docs/intro" className={styles.startButton}>
          Start Reading
        </Link>
      </div>
      {modules.map((module, index) => (
        <ModuleCard
          key={index}
          moduleName={module.name}
          chapters={module.chapters}
          icon={module.icon}
        />
      ))}
    </div>
  );
};

export default LandingPage;