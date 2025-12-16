// src/components/Shared/ModuleCard.tsx
import React from 'react';
import styles from './ModuleCard.module.css'; // Assuming CSS Modules

interface ModuleCardProps {
  moduleName: string;
  chapters: string[];
  icon: string; // Can be an emoji or an image path
}

const ModuleCard: React.FC<ModuleCardProps> = ({ moduleName, chapters, icon }) => {
  const isImagePath = icon.startsWith('/'); // Simple check for image path

  return (
    <div className={styles.moduleCard}>
      <div className={styles.moduleHeader}>
        {isImagePath ? (
          <img src={icon} alt={`${moduleName} icon`} className={styles.moduleImage} />
        ) : (
          <span className={styles.moduleIcon}>{icon}</span>
        )}
        <h3>{moduleName}</h3>
      </div>
      <ul className={styles.chapterList}>
        {chapters.map((chapter, index) => (
          <li key={index}>{chapter}</li>
        ))}
      </ul>
    </div>
  );
};

export default ModuleCard;