<template>
  <div id="app">
    <control></control>
    <ros></ros>
  </div>
</template>

<script>
import ROS from './components/ROS.vue'
import Control from './components/Control.vue'

export default {
  name: 'app',
  components: {
    'ros': ROS,
    'control': Control,
  }
}
</script>

<style>
</style>
