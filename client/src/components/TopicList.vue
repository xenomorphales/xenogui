<template>
  <div id="topiclist">
    <mt-cell v-for="topic in topics" :key="topic.name" :title="topic.name">{{topic.namespace}}{{topic.name}}</mt-cell>
  </div>
</template>

<script>
export default {
  name: 'topiclist',
  data() {
      return {
          topics: [
              {'name':'topic1', 'namespace':'/'},
              {'name':'topic2', 'namespace':'/'},
              {'name':'topic3', 'namespace':'/'}
          ]
      }
  }
}
</script>

<style>
</style>
