<template>
  <div id="ros">
    <mt-navbar v-model="selected">
      <mt-tab-item id="nodes">Nodes</mt-tab-item>
      <mt-tab-item id="topics">Topics</mt-tab-item>
    </mt-navbar>
    <mt-tab-container v-model="selected">
      <mt-tab-container-item id="nodes">
        <nodelist></nodelist>
      </mt-tab-container-item>
      <mt-tab-container-item id="topics">
        <topiclist></topiclist>
      </mt-tab-container-item>
    </mt-tab-container>
  </div>
</template>

<script>
import NodeList from './NodeList.vue'
import TopicList from './TopicList.vue'

export default {
  name: 'ros',
  data() {
    return { selected: 'nodes' }
  },
  components: {
    'nodelist': NodeList,
    'topiclist': TopicList,
  }
}
</script>

<style>
</style>
