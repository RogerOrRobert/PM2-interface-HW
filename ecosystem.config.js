module.exports = {
    apps: [
      ...require('./apps/admin-api/pm2.config.js').apps,
      ...require('./apps/api/pm2.config.js').apps,
      ...require('./apps/proxy/pm2.config.js').apps,
      ...require('./apps/next/pm2.config.js').apps,
      ...require('./apps/admin/pm2.config.js').apps,
     /*  {
        name: 'app.py',
        script: './python-scripts/app.py',
        interpreter: 'C:/Python311/python.exe', //'/usr/bin/python',
        exec_mode: 'fork',
        error_file: './pm2_mqttError.log'
      } */
    ]
};