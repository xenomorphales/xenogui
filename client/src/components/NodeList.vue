<template>
  <div id="nodelist">
    <mt-cell v-for="node in nodes" :key="node" :title="node">{{node}}</mt-cell>
  </div>
</template>

<script>
export default {
  name: 'nodelist',
  data() {
      return {
          nodes: [
              'none',
          ]
      }
  },
  mounted() {
      var _this = this;
      setInterval(function() {
          _this.$http.get('/nodes').then(response => {
              _this.nodes = response.body;
          });
      }, 1000);
  }
}
</script>

<style>
</style>
