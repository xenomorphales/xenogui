import Vue from 'vue'

import MintUI from 'mint-ui'
import 'mint-ui/lib/style.css'

import VueResource from 'vue-resource'

import App from './App'

Vue.config.productionTip = false

Vue.use(MintUI)
Vue.use(VueResource)

new Vue({
  el: '#app',
  template: '<App/>',
  components: { App }
})
