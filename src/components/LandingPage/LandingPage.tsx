// src/components/LandingPage/LandingPage.tsx
import React from 'react';
import Link from '@docusaurus/Link';
import styles from './LandingPage.module.css'; // Assuming CSS Modules

const LandingPage = () => {
  return (
    <div className={styles.landingPageContainer}>
      <div className={styles.centerSection}>
        <h1 className={styles.bookTitle}>AI-Spec–Driven Technical Book</h1>
        <p className={styles.bookDefinition}>Physical AI & Humanoid Robotics with Integrated RAG Chatbot</p>
        <Link to="/docs/intro" className={styles.startButton}>
          Start Reading
        </Link>
      </div>
    </div>
  );
};

export default LandingPage;