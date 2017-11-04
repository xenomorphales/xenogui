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
                {'name':'none','type':undefined}
            ]
        }
    },
    mounted() {
        this.socket = new WebSocket("ws://"+window.location.hostname+":"+window.location.port+"/websocket");
        var _this = this;
        this.socket.onmessage = function(evt) {
            var msg = JSON.parse(evt.data);
            if(msg.event == 'list') {
                _this.topics = msg.data;
            }
            if(msg.event == 'recv') {
                var topics = JSON.parse(JSON.stringify(_this.topics));
                for(var t in topics) {
                    if(topics[t].name == msg.data.name) {
                        topics[t].data = msg.data.msg;
                    }
                }
                _this.topics = topics;
            }
        };
    }
}
</script>

<style>
</style>
