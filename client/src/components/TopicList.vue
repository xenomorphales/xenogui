<template>
  <div id="topiclist">
    <mt-cell v-for="topic in topics" :key="topic.name" :title="topic.name">{{topic.data}}</mt-cell>
  </div>
</template>

<script>
export default {
  name: 'topiclist',
  data() {
      return {
          topics: [
              {'name':'none','type':undefined, 'data':undefined}
          ]
      }
  },
  mounted() {
      var _this = this;
      setInterval(function() {
          _this.$http.get('/topics').then(response => {
              var topics = response.body;
              var ok = 0;
              for(var t in topics) {
                  _this.$http.get('/topic/'+topics[t].name).then(response => {
                      topics[t].data = response.body;
                      ok++;
                  }, response => { ok++; });
              }
              setTimeout(function() {
                  _this.topics = topics;
              }, 1000);
          });
      }, 1000);
  }
}
</script>

<style>
</style>
