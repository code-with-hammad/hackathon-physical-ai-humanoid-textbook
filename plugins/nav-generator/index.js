const fs = require('fs');
const path = require('path');
const matter = require('gray-matter');

module.exports = function (context, options) {
  return {
    name: 'docusaurus-nav-generator-plugin',
    async createContent() {
      const docsPath = path.join(context.siteDir, 'docs');
      const outputPath = path.join(
        context.siteDir,
        'src',
        'components',
        'PremiumLandingPage',
        'nav-data.json'
      );

      try {
        const modules = fs
          .readdirSync(docsPath, { withFileTypes: true })
          .filter((dirent) => dirent.isDirectory())
          .map((dir) => {
            const modulePath = path.join(docsPath, dir.name);
            const categoryFilePath = path.join(modulePath, '_category_.json');
            let moduleTitle = dir.name;

            // Try to get title from _category_.json
            if (fs.existsSync(categoryFilePath)) {
              const categoryContent = fs.readFileSync(categoryFilePath, 'utf8');
              const categoryData = JSON.parse(categoryContent);
              if (categoryData.label) {
                moduleTitle = categoryData.label;
              }
            }

            const chapters = fs
              .readdirSync(modulePath)
              .filter((file) => file.endsWith('.md') || file.endsWith('.mdx'))
              .map((file) => {
                const filePath = path.join(modulePath, file);
                const fileContent = fs.readFileSync(filePath, 'utf8');
                const { data } = matter(fileContent); // Parse front matter

                // Use front matter title or format the file name
                const chapterTitle =
                  data.title ||
                  path.basename(file, path.extname(file)).replace(/-/g, ' ');
                const docPath = path.join(
                  '/docs',
                  dir.name,
                  path.basename(file, path.extname(file))
                );

                return {
                  title: chapterTitle,
                  path: docPath,
                };
              });

            return {
              title: moduleTitle,
              icon: 'placeholder.svg', // As per data-model
              chapters: chapters,
            };
          });

        fs.writeFileSync(outputPath, JSON.stringify(modules, null, 2));
        console.log('✅ Navigation data generated successfully.');
      } catch (error) {
        console.error('❌ Error generating navigation data:', error);
        // We will generate an empty file on error to prevent build crashes
        // The component should handle the case where the data is empty
        fs.writeFileSync(outputPath, JSON.stringify([], null, 2));
      }
    },
  };
};
