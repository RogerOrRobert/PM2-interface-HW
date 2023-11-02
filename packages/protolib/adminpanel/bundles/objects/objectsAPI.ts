import { ObjectModel } from ".";
import { CreateApi, getImport, getDefinitions, getSourceFile, extractChainCalls, addImportToSourceFile, ImportType, addObjectLiteralProperty, getFirstDefinition } from '../../../api'
import { promises as fs } from 'fs';
import * as fspath from 'path';
import { Project, SyntaxKind, ObjectLiteralExpression, PropertyAssignment } from 'ts-morph';
import axios from 'axios';

const PROJECT_WORKSPACE_DIR = process.env.FILES_ROOT ?? "../../";
const indexFile = "/packages/app/bundles/custom/schemas/index.ts"

const getSchemas = async (sourceFile?) => {
  const definition = getFirstDefinition(sourceFile ?? getSourceFile(fspath.join(PROJECT_WORKSPACE_DIR, indexFile)), '"schemas"')

  if (definition) {
    const schemas = []
    const node = definition.getArguments()[1]
    if (node instanceof ObjectLiteralExpression) {
      node.getProperties().forEach(prop => {
        if (prop instanceof PropertyAssignment) {
          // obj[prop.getName()] = prop.getInitializer().getText();
          schemas.push({ name: prop.getName(), id: prop.getInitializer().getText() }) //= prop.getInitializer().getText();
        }
      });
    }
    // console.log(schemas)
    return schemas
  }
  return []
}

const getSchema = async (idSchema) => {
  let SchemaFile = fspath.join(PROJECT_WORKSPACE_DIR, indexFile)
  let sourceFile = getSourceFile(SchemaFile)
  const schemas = await getSchemas(sourceFile)
  const currentSchema = schemas.find(s => s.id == idSchema)

  sourceFile = getSourceFile(fspath.join("../../packages/app/bundles/custom/schemas/", getImport(sourceFile, idSchema)) + ".ts")
  const definition = getFirstDefinition(sourceFile, '"schema"')
  let keys = {}
  if (definition) {
    const node = definition.getArguments()[1]
    if (node instanceof ObjectLiteralExpression) {
      node.getProperties().forEach(prop => {
        if (prop instanceof PropertyAssignment) {
          // obj[prop.getName()] = prop.getInitializer().getText();
          const chain = extractChainCalls(prop.getInitializer())
          if (chain.length) {
            const typ = chain.shift()
            keys[prop.getName()] = {
              type: typ.name,
              params: typ.params,
              modifiers: chain
            }
          }
        }
      });
    }
  }
  return { name: currentSchema.name, id: idSchema, keys: keys }
}

const setSchema = (path, content, value) => {
  let sourceFile = getSourceFile(path)
  const definition = getFirstDefinition(sourceFile, '"schema"')
  if (!definition) {
    throw "No schema marker found for file: " + path
  }
  console.log('definitions: ', definition)

  if (definition.getArguments().length > 1) {
    const secondArgument = definition.getArguments()[1];
    secondArgument.replaceWithText(content);
  }

  sourceFile.saveSync();

  //link in index.ts
  sourceFile = getSourceFile(fspath.join(PROJECT_WORKSPACE_DIR, indexFile))

  addImportToSourceFile(sourceFile, value.id, ImportType.NAMED, './' + value.name)

  const linkDefinition = getFirstDefinition(sourceFile, '"schemas"')
  if (!linkDefinition) {
    throw "No link definition schema marker found for file: " + path
  }

  if (linkDefinition.getArguments().length > 1) {
    const secondArgument = linkDefinition.getArguments()[1];
    console.log('second argument: ', secondArgument)
    //secondArgument.replaceWithText(content);
    addObjectLiteralProperty(secondArgument, value.name, value.id)
  }
  sourceFile.saveSync();
}

const getDB = (path, req, session) => {
  const db = {
    async *iterator() {
      const schemas = await getSchemas();
      for (const schema of schemas) {
        yield [schema.name, JSON.stringify(schema)];
      }
    },

    async put(key, value) {
      value = JSON.parse(value)
      let exists
      const filePath = PROJECT_WORKSPACE_DIR + 'packages/app/bundles/custom/schemas/' + value.name.replace(/[^a-zA-Z0-9_.-]/g, '') + '.ts'
      try {
        await fs.access(filePath, fs.constants.F_OK)
        exists = true
      } catch (error) {
        exists = false
      }

      if (exists) {
        console.log('File: ' + filePath + ' already exists, not executing template')
      } else {
        await axios.post('http://localhost:8080/adminapi/v1/templates/file', {
          name: value.name + '.ts',
          data: {
            options: { template: '/packages/protolib/adminpanel/bundles/objects/templateSchema.tpl', variables: { name: value.name.charAt(0).toUpperCase() + value.name.slice(1) } },
            path: '/packages/app/bundles/custom/schemas'
          }
        })
      }

      const result = "{" + Object.keys(value.keys).reduce((total, current, i) => {
        const v = value.keys[current]
        const modifiers = v.modifiers ? v.modifiers.reduce((total, current) => total + '.' + current.name + "(" + (current.params && current.params.length ? current.params.join(',') : '') + ")", '') : ''
        return total + "\n\t" + current + ": " + "z." + v.type + "(" + (v.params && v.params.length ? v.params.join(',') : '') + ")" + modifiers + ","
      }, '').slice(0, -1) + "\n}"


      await setSchema(filePath, result, value)
    },

    async get(key) {
      return JSON.stringify(await getSchema(key))
    }
  };

  return db;
}

export const ObjectsAPI = (app) => CreateApi('objects', ObjectModel, __dirname, '/adminapi/v1/', '', {}, () => { }, getDB)(app)