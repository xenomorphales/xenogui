<template>
  <div id="nodelist">
    <mt-cell v-for="node in nodes" :key="node.name" :title="node.name">{{node.namespace}}{{node.name}}</mt-cell>
  </div>
</template>

<script>
export default {
  name: 'nodelist',
  data() {
      return {
          nodes: [
              {'name':'node1',  'namespace':'/'},
              {'name':'node2',  'namespace':'/'},
              {'name':'mynode', 'namespace':'/'},
              {'name':'node3',  'namespace':'/'}
          ]
      }
  }
}
</script>

<style>
</style>
