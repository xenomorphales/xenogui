<template>
  <div id="topiclist">
    <mt-cell v-for="topic in topics" :key="topic.name" :title="topic.name">{{topic.type}}</mt-cell>
  </div>
</template>

<script>
export default {
  name: 'topiclist',
  data() {
      return {
          topics: [
              {'name':'none','type':undefined}
          ]
      }
  },
  mounted() {
      var _this = this;
      setInterval(function() {
          _this.$http.get('/topics').then(response => {
              _this.topics = response.body;
          });
      }, 1000);
  }
}
</script>

<style>
</style>
