module.exports = function (context, options) {
  return {
    name: 'thebe-docusaurus-plugin',
    injectHtmlTags() {
      return {
        headTags: [
          {
            tagName: 'script',
            attributes: {
              src: 'https://unpkg.com/thebe@latest/dist/index.js',
            },
          },
        ],
      };
    },
  };
};
