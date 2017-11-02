<template>
  <div id="control">
    <mt-cell :title="side" :class="side"><mt-switch v-model="switch_side"></mt-switch></mt-cell>
    <mt-progress :value="match_time" :bar-height="10"></mt-progress>
    <mt-button size="large" :type="but_type" @click="toggle_match">{{but_text}}</mt-button>
  </div>
</template>

<script>
export default {
  name: 'nodelist',
  data() {
      return {
          switch_side: false,
          match_time: 0,
          match_started: false,
          match_timer: undefined,
      }
  },
  computed: {
      side() {
          if(this.switch_side) return 'orange';
          else                 return 'green';
      },
      but_type() {
          if(this.match_started) return 'danger';
          else                   return 'primary';
      },
      but_text() {
          if(this.match_started) return 'Stop';
          else                   return 'Start';
      },
  },
  methods: {
      toggle_match() {
          if(this.match_started) {
              this.match_started = false;
              this.match_time = 0;
              clearInterval(this.match_timer);
              this.match_timer = undefined;
          }
          else {
              this.match_started = true;
              self = this;
              this.match_timer = setInterval(function() {
                  self.match_time += 1;
                  if(self.match_time >= 100.0) {
                    self.match_started = false;
                    self.match_time = 0;
                    clearInterval(self.match_timer);
                    self.match_timer = undefined;
                  }
              },100)
          }
      }
  }
}
</script>

<style>
.orange { background-color: orange; }
.green { background-color: green; }
</style>
