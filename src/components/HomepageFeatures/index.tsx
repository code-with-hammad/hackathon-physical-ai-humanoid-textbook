import React from 'react';
import styles from './index.module.css';

const modules = [
  {
    title: 'Introduction',
    description: 'Basics of AI-Native Development',
    img: '/images/robot_intro.png',
  },
  {
    title: 'Humanoids',
    description: 'AI Integration with Humanoids',
    img: '/images/robot_humanoid.png',
  },
  {
    title: 'Advanced Techniques',
    description: 'Deep dive into AI modules',
    img: '/images/robot_advanced.png',
  },
  {
    title: 'Projects',
    description: 'Hands-on projects and examples',
    img: '/images/robot_projects.png',
  },
];

export default function ModulesSection() {
  return (
    <div className={styles.modulesSection}>
      <h2 className={styles.modulesHeading}>Explore Modules</h2>
      <div className={styles.tabsContainer}>
        {modules.map((mod, idx) => (
          <div key={idx} className={styles.tabCard}>
            <img src={mod.img} alt={mod.title} className={styles.moduleImg} />
            <h3>{mod.title}</h3>
            <p>{mod.description}</p>
          </div>
        ))}
      </div>
    </div>
  );
}
