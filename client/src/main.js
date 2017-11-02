import Vue from 'vue'

import MintUI from 'mint-ui'
import 'mint-ui/lib/style.css'

import App from './App'

Vue.config.productionTip = false

Vue.use(MintUI)

new Vue({
  el: '#app',
  template: '<App/>',
  components: { App }
})
