document.addEventListener('DOMContentLoaded', () => {
    var control_ajax;
    var action_ajax;
    var cache_bar={};
    const motor_table = document.getElementById('motor_config_tb');
    const control_table = document.getElementById('control_config_tb');
    const control_page = document.getElementById('control_center');
    const modify_motor_modal=document.getElementById("motor-modify-modal");
    const modify_control_modal=document.getElementById("control-modify-modal");
    const add_action_group_modal=document.getElementById(`add-action-group-modal`);
    const add_action_modal= document.getElementById(`add-action-modal`);
    const action_group_details=document.getElementById('action_group_details');
    const default_current=5.0
    const default_speed=0.1
    const card_strech_time=1000;
    const ip = "192.168.31.40"
    const url=`http://${ip}:3754`;
    var imu_socket;
    var motor_data
    var motor_socket = new WebSocket(`ws://${ip}:8765`); // 修改为你的WebSocket服务器地址
    // var imu_ws = new WebSocket(`ws://${ip}:1100`);
    imu_ws=null

    motor_socket.onopen = function() {
        console.log('Connected to the server');
    };

    motor_socket.onmessage = function(event) {
        motor_data=JSON.parse(event.data)
        motor_data.forEach(function(data,index){
            const check_box=$(`#control-checkbox-${data[0]}`)
            const loading=$(`#control_bar_loading_${data[0]}`)
            if(data[1]===0 && check_box){
                check_box.attr("disabled",true)
                check_box.css("display","block")
                loading.css("display","none")
            }
            if(data[1]===1 && check_box){
                check_box.attr("disabled",false)
                check_box.css("display","block")
                loading.css("display","none")
            }
            if(data[1]===2 && check_box){
                check_box.css("display","none")
                loading.css("display","block")
            }
        })
    };

    motor_socket.onerror = function(error) {
        console.error('WebSocket Error:', error);
    };

    motor_socket.onclose = function(e){
        motor_socket = new WebSocket(`ws://${ip}:8765`);
        console.log("reconnect motor_websocket")
    }

    // imu_ws.onmessage = function(event) {
    //     const angles = JSON.parse(event.data);
    //     document.getElementById('roll').textContent = angles[0].toFixed(2);
    //     document.getElementById('pitch').textContent = angles[1].toFixed(2);
    //     document.getElementById('yaw').textContent = angles[2].toFixed(2);
    // };

    // imu_ws.onerror = function(error) {
    //     console.error('WebSocket 错误:', error);
    // };

    // imu_ws.onclose = function() {
    //     console.log('WebSocket 连接关闭');
    // };

    function imu_angle_show(){
        var imu_ws = new WebSocket(`ws://${ip}:1100`);
        imu_ws.addEventListener('open', (event) => {
            console.log('websocket连接已建立');
        })
        socket.addEventListener('message', (event) => {
            console.log(`收到: ${event.data}`);
            const angles = JSON.parse(event.data);
            document.getElementById('roll').textContent = angles[0].toFixed(2);
            document.getElementById('pitch').textContent = angles[1].toFixed(2);
            document.getElementById('yaw').textContent = angles[2].toFixed(2);
        });
        socket.addEventListener('close', (event) => {
            isConnected = false;
            if (event.wasClean) {
                addMessageToLog(`连接已关闭，状态码: ${event.code}，原因: ${event.reason || '无'}`, 'system');
            } else {
                addMessageToLog('连接异常断开', 'error');
            }
        });
    }

    function modify_motor_window_open(motor_id){
        $.ajax({
            type: "get",
            url: "/motor/get_a_motor",
            data: {"motor_id":parseInt(motor_id)},
            success: function (response) {
                if(response.motor!==null){
                    const motor=response.motor
                    $(`#modify_motor`).attr("data",motor_id)
                    $(`#modify_motor_id`).val(motor[8])
                    $(`#modify_can_id`).val(motor[1])
                    $(`#modify_motor_name`).val(motor[2])
                    $(`#modify_motor_current_position`).val(motor[3])
                    $(`#modify_motor_max_position`).val(motor[4])
                    $(`#modify_motor_min_position`).val(motor[5])
                    $(`#modify_motor_default_position`).val(motor[6])
                    $(`#modify_protocol`).val(motor[7])
                }     
            }
        });
        modify_motor_modal.style.display = "block";
    }

    function modify_control_window_open(element){
        $('.union_control').remove();
        const targetElement = element.parentNode.parentNode

        // 获取所有单元格元素
        var cells = targetElement.getElementsByTagName("td");
        // 创建一个数组来存储单元格内容
        var cellContents = [];
        // 遍历每个单元格并提取其文本内容
        for (var i = 0; i < cells.length-2; i++) {
            // 将单元格内的文本内容添加到数组中
            cellContents.push(cells[i].textContent.trim());
        }
        console.log(cellContents)
        const motor_select = document.getElementById('modify_motor_select');
        const choose_motor_output = document.getElementById('modify_choose_motor_output');
        let control_mode=1
        if(cellContents[4]==="位置/速度模式"){
            control_mode=1
        }
        if(cellContents[4]==="力矩/位置模式"){
            control_mode=2
        }
        $.ajax({
            type:"get",
            url:"/controller/get_motor_id",
            data:{"control_id":parseInt(cellContents[0])},
            success:function(response){
                const motor_id=response.controller
                update_controller_form(motor_select,choose_motor_output,"modify_",motor_id,response.location[0])
                modify_control_form(cellContents[3],"modify");
                $("#modify_target").text(`修改${cellContents[0]}号控制节点`);
                $(`#modify_joint_name`).val(cellContents[1])
                $(`#modify_control_type`).val(cellContents[3])
                $(`#modify_control_mode`).val(control_mode)
                if(cellContents[3]==="2"){
                    $(`#modify_control_current_position`).val(cellContents[5])
                    $(`#modify_control_max_position`).val(cellContents[6])
                    $(`#modify_control_min_position`).val(cellContents[7])
                    $(`#modify_control_default_position`).val(cellContents[8])
                }
                modify_control_modal.style.display="block"
            }
        })
    }

    function add_actions_window_open(){
        const actions=get_control_form_action()
        console.log(actions)
        if(actions.length===0){
            alert("请先勾选需要添加的动作")
            return
        }
        $(`#add_action_seq`).val(null)
        update_action_form(actions,"add")
        $.ajax({
            type:"get",
            url:"/action_group/get_all",
            data:"",
            success:function(response){
                if(response.groups){
                    response.groups.forEach(function(group,index){
                        $(`#add_action_select_group`).append(`<option value="${group[0]}">${group[1]}</option>`)
                    })
                }
                add_action_modal.style.display="block";
            },
            error: function(xhr, status, error) {
                console.error("An error occurred: " + error);
                add_action_modal.style.display="block";
            }
        })
    }

    function add_actions_group_window_open(){
        add_action_group_modal.style.display="block"
    }

    document.querySelectorAll('#motor_config_tb').forEach(container => {
        container.addEventListener('click', (event) => {
            if (event.target && event.target.classList.contains('table_operation_btn_waring')) {
                const motorId = event.target.getAttribute('data-motor-delete-id');
                delete_motor(motorId);
            }
            if (event.target && event.target.classList.contains('table_operation_btn')) {
                const motorId = event.target.getAttribute('data-motor-modify-id');
                modify_motor_window_open(motorId);
            }
        });
    });

    document.querySelectorAll('#actions_table').forEach(container => {
        container.addEventListener('click', (event) => {
            if (event.target && event.target.classList.contains('table_operation_btn_waring')){
                const action_id = event.target.getAttribute('data');
                $.ajax({
                    type: "delete",
                    url: `/action/delete`,
                    data: {"action_id":action_id},
                    success: function (response) {
                        if(response.message==="success"){
                            const group_id=$(`#action-group-name`).attr("data");
                            group_detail_on(group_id);
                        }
                    }
                });
            }
            if (event.target && event.target.classList.contains(`table_operation_btn`)){
                const action_id = event.target.getAttribute(`data`);
                $.ajax({
                    type: "get",
                    url: "/action/get",
                    data: {"action_id":action_id},
                    success: function (response) {
                        $(`#modify_actionform input`).val(null)
                        $(`#modify_control_bar .card_body`).remove()
                        $(`#modify_action_name`).val(response.action[2])
                        $(`#modify_action_seq`).val(response.action[3])
                        console.log(response.action)
                        response.action[4].forEach((data,index)=>{
                            $(`#modify_control_bar`).append(`
                                <div class="form-group card_body" style="margin-bottom: 2%;" data="${response.action[0]}">
                                    <label for="modify_control-${data.motor_id}" style="width:25%">${data.location}|${data.joint_name}:</label>
                                    <input type="range" class="modify-form-control-range" id="modify_control-${data.motor_id}" min="${data.min}" max="${data.max}" value="${data.position}" step="${(data.max-data.min)/100}"/>
                                    <input type="number" class="modify-control-position-input" style="width:10%" value="${data.position}" id="modify_control-position-${data.motor_id}"
                                        oninput="if(value>${data.max})value=${data.max};if(value<${data.min})value=${data.min}" step="${(data.max-data.min)/100}"/>
                                    <label for="modify_control-speed-${data.motor_id}" style="width:10%">速度:</label>
                                    <input type="number" style="width:10%" value="${data.speed}" id="modify_control-speed-${data.motor_id}" min="0" step="0.1" name_index="${data.name_index}"/>
                                    <button class="table_operation_btn_waring" style="width: 30px;" data="${data.motor_id}">x</button>
                                </div>
                            `)
                        })
                        $(`#modify-action-modal`).css("display","block");
                        $(`#modify-action-modal`).attr("data",action_id)    
                    }
                });
            }
            if(event.target && event.target.classList.contains('button_seq')){
                const way=event.target.getAttribute("way")
                const action_id=event.target.getAttribute("data")
                const group_id=$(`#action-group-name`).attr("data")
                $.ajax({
                    type: "post",
                    url: `/action/seq_${way}`,
                    data: {"action_id":parseInt(action_id),"group_id":parseInt(group_id)},
                    success: function (response) {
                        if(response.message==="success"){
                            const group_id=$(`#action-group-name`).attr("data")
                            group_detail_on(group_id)
                        }
                    }
                });
            }
            if (event.target && event.target.classList.contains('action_command')){
                let command_str=event.target.textContent
                jsonStr = command_str.replace(/None/g, 'null'); // 替换所有单引号为双引号
                console.log(jsonStr)
                const command = JSON.parse(jsonStr);
                action_run(command)
            }
        });
    });

    document.querySelectorAll(`#action_groups_cards_box`).forEach(container => {
        container.addEventListener('click', (event) => {
            if (event.target && event.target.classList.contains('btn-danger')) {
                const group_id=event.target.getAttribute("data")
                $.ajax({
                    type: "delete",
                    url: "/action_group/delete",
                    data: {"group_id":parseInt(group_id)},
                    success: function (response) {
                        if(response.message==="success"){
                            refresh_action_group()
                            group_detail_off()
                        }
                    }
                });
            }
            if (event.target && event.target.classList.contains('edit')) {
                $(`#modify-action-group-modal`).css("display","flex")
                const group_id = event.target.getAttribute('data');
                $(`#modify_action_group`).attr("data",group_id)
                $.ajax({
                    type: "get",
                    url: "/action_group/get_group",
                    data: {"group_id":group_id},
                    success: function (response) {
                        console.log(response)
                        if(response){
                            $(`#modify_action_group_name`).val(response[1])
                            if(response[2]!==null){
                                $(`#modify_action_group_callback`).val(response[2])
                            }
                            $(`#modify_action_group_description`).val(response[3])
                        }
                    }
                });
            }
        })
    })

    function modify_control_form(control_type,modify_){
        let form_type=""
        if (modify_==="modify"){
            form_type="modify_"
        }
        if (modify_==="add"){
            form_type=""
        }
        if (control_type==="2"){
            $(`#${modify_}_controller_form`).append(
                `
                <div class="form_box union_control">
                    <label for="${form_type}control_current_position">当前位置</label>
                    <input type="number" class="form-control" id="${form_type}control_current_position" placeholder="当前位置">
                </div>
                <div class="form_box union_control">
                    <label for="${form_type}control_max_position">最大位置</label>
                    <input type="number" class="form-control" id="${form_type}control_max_position" placeholder="最大位置">
                </div>
                <div class="form_box union_control">
                    <label for="${form_type}control_min_position">最小位置</label>
                    <input type="number" class="form-control" id="${form_type}control_min_position" placeholder="最小位置">
                </div>
                <div class="form_box union_control">
                    <label for="${form_type}control_default_position">默认位置</label>
                    <input type="number" class="form-control" id="${form_type}control_default_position" placeholder="默认位置">
                </div>`
            )
        }
        else{
            $('.union_control').remove();
        }
    }

    document.querySelectorAll('#control_config_tb').forEach(container => {
        container.addEventListener('click', (event) => {
            if (event.target && event.target.classList.contains('table_operation_btn_waring')) {
                const controlId = event.target.getAttribute('data-control-delete-id');
                const motor_id=event.target.getAttribute("motor_id")
                delete_control(controlId,motor_id);
            }
            if (event.target && event.target.classList.contains('table_operation_btn')) {
                const controlId = event.target.getAttribute('data-control-modify-id');
                modify_control_window_open(event.target);
            }
            if (event.target.matches('[id^="depends-a-"]')) {
                const numbers = event.target.id.match(/\d+/g); 
                $(`#depends-upload-${numbers[0]}-${numbers[1]}`).click()
            }
        });
        container.addEventListener("change", (change_event) => {
            if (change_event.target.matches(`[id^="depends-upload-"]`)) {
                const numbers = change_event.target.id.match(/\d+/g); 
                if (change_event.target.files.length > 0) {
                    const file = change_event.target.files[0];
                    if (file.name.endsWith('.pkl')) {
                        $(`#depends-a-${numbers[0]}-${numbers[1]}`).text(file.name);
                        var formData = new FormData();
                        formData.append("file", file);
                        formData.append("control_id", numbers[0]);//此处需要加入motor_id
                        formData.append("motor_id",numbers[1])
                        $.ajax({
                            url: "/controller/depend/add",
                            type: "put",
                            data: formData,
                            processData: false,
                            contentType: false,
                            success: function(response) {
                                console.log("File uploaded successfully: " + response.filename);
                            },
                            error: function(xhr, status, error) {
                                console.error("An error occurred: " + error);
                            }
                        });
                    } else {
                        alert("文件不合规，请上传 .pkl 格式的文件");
                        change_event.target.value = ''; // 清空文件输入框
                    }
                } else {
                    alert("文件不合规");
                }
            }
        });

    });

    function update_control_bar(element){
        if (parseInt(element.getAttribute("control_type")) === 2){
            let temp = element.getAttribute("id").split("-")
            let control_id = parseInt(temp[temp.length - 1])
            console.log("旧"+String(cache_bar[String(control_id)]))
            console.log("新"+String(element.value))
            let form = {
                "control_id": control_id,
                "value": String(element.value-cache_bar[String(control_id)]),
                "data": []
            }
            let reflect = element.getAttribute("reflect").split(",")
            if (reflect !== null){
                for (let index = 0; index < reflect.length; index++) {
                    console.log($(`#control-${reflect[index]}`))
                    let temp_dict = {}
                    temp_dict["id"] = parseInt(reflect[index])
                    temp_dict["motor_id"] = parseInt($(`#control-${reflect[index]}`).attr("motor_ids"))
                    temp_dict["position"] = String($(`#control-${reflect[index]}`).val())
                    form["data"].push(temp_dict)
                }
            }
            cache_bar[String(control_id)]=element.value
            $.ajax({
                type: "post",
                url: "/control_bar/update",
                contentType: "application/json",
                data: JSON.stringify(form),
                success: function (response) {
                    console.log(response)
                    response.message.forEach(function(data){
                        $(`#control-${String(data.control_id)}`).val(data.position)
                        $(`#control-position-${String(data.control_id)}`).val(data.position)
                    })
                }
            });
        }
    }
    

    document.querySelectorAll(`#control_center`).forEach(container=> {
        container.addEventListener('click', (event)=>{
            if (event.target && event.target.classList.contains(`controller_running`)){
                const control_id=event.target.getAttribute("data")
                $.ajax({
                    type: "put",
                    url: "/control/stop_waiting",
                    data: {"control_id":parseInt(control_id)},
                    success: function (response) {
                        console.log("Stopping waiting control down")
                    }
                });
            }
        })
        container.addEventListener("change",(event=>{
            if (event.target && event.target.classList.contains("location_all_select")){
                const checkboxes=event.target.parentNode.parentNode.parentNode.querySelectorAll('input[type="checkbox"]')
                console.log(checkboxes)
                if(event.target.checked){
                    checkboxes.forEach(function(checkbox) {
                        checkbox.checked=true
                    });
                }
                else{
                    checkboxes.forEach(function(checkbox) {
                        checkbox.checked=false
                    });
                }
            }
            if (event.target && (event.target.classList.contains("form-control-range"))){
                update_control_bar(event.target)
            }
        }))
        container.addEventListener('change', (event)=>{
            if(event.target && event.target.classList.contains("control-position-input")){
                update_control_bar(event.target)
            }
        })
    })

    function get_controller_form(motor_selects,control_type,form_type){
        const numbers=[]
        const control_id=$(`#${form_type}control_id`).val()
        motor_selects.each(function(index,data){
            number=$(data).text().match(/(\d+)$/)
            if (number) {
                // 将提取到的数字添加到数组中
                numbers.push(parseInt(number[0], 10));
            }
        })
        const loaction=$(`#${form_type}location`).val()
        let description=$(`#${form_type}control_description`).val()
        if(loaction===""){
            alert("请选择该关节所处位置")
            return false
        }
        if(description==""){
            description="该用户很懒，没有留下任何备注"
        }
        let data={
            "control_id":$(`#${form_type}control_id`).val(),
            "name":$(`#${form_type}joint_name`).val(),
            "location":loaction,
            "move_type":parseInt($(`#${form_type}control_type`).val()),
            "motor_id":String(numbers),
            "description":description
        }

        if(control_type==="0"){
            alert("请选择控制类型")
            return false
        }
        if(control_type==="1"){
            Object.assign(data,{
                "control_mode":1
            })
        }
        if(control_type==="2"){
            Object.assign(data,{
                "control_mode":parseInt($(`#${form_type}control_mode`).val()),
                "current_position":$(`#${form_type}control_current_position`).val(),
                "max_position":$(`#${form_type}control_max_position`).val(),
                "min_position":$(`#${form_type}control_min_position`).val(),
                "default_position":$(`#${form_type}control_default_position`).val(),
                "depends":$(`#${form_type}depends`).val()
            })
        }
        return data
    }

    function get_control_form_action(){
        const motor_id_checkbox=$('input[type="checkbox"][id^="control-checkbox-"]').filter(':checked');
        let data=[]
        motor_id_checkbox.each(function() {
            let action={}
            action["motor_id"]=$(this).attr('id').split("-")[2]
            action["location"] = $(this).parent().siblings('.card_title').children('label').text()
            action["name_index"]=$(this).attr("name_index")
            action["position"] = $(`#control-position-${action["motor_id"]}`).val()
            action["speed"] = $(`#control-speed-${action["motor_id"]}`).val()
            data.push(action)
        })
        return data
    }
    
    function get_add_action_group_form(form_type){
        let data={}
        data.name=$(`#${form_type}_action_group_name`).val()
        temp=$(`#${form_type}_action_group_callback`).val()
        if(temp){
            data.action_callback=temp
        }                   
        data.description=$(`#${form_type}_action_group_description`).val()
        return data
    }

    function get_all_locations(form_type,location_value){
        console.log(form_type)
        $.ajax({
            type: "get",
            url: "/controller/get_locations",
            success: function (response) {
                response.forEach(function(data,index){
                    $(`#${form_type}location`).append(`
                        <option value='${data[0]}'>${data[1]}-${data[2]}模块</option>    
                    `)
                })
                $(`#${form_type}location`).val(location_value)
            }
        });
    }

    function update_controller_form(motor_select,choose_motor_output,form_type,append_motor_id,location_value){
        const location=document.getElementById(`${form_type}location`)
        while (motor_select.firstChild) {
            motor_select.removeChild(motor_select.firstChild);
        }
        while (choose_motor_output.firstChild) {
            choose_motor_output.removeChild(choose_motor_output.firstChild);
        }
        while (location.firstChild) {
            location.removeChild(location.firstChild);
        }
        motor_select.insertAdjacentHTML('beforeend',`<option value="0" selected >请选择选择电机</option>`)
        location.insertAdjacentHTML('beforeend',`<option value="" selected >请选择关节所属位置</option>`)
        get_all_locations(form_type,location_value)
        $.ajax({
            type:"get",
            url:"/motor/get_all",
            data:"",
            success: function(data) { // 请求成功时的回调函数
                if (data.motors){
                    data.motors.forEach((motor, index) => {
                        motor_select.insertAdjacentHTML('beforeend',`<option value="${motor[0]}">${motor[2]}-${motor[0]}</option>`)
                        if(form_type==="modify_" &&  append_motor_id.includes(motor[0])){
                            const newOptionDiv = document.createElement('div');
                            newOptionDiv.className = 'motor_choose_output_div';
                            newOptionDiv.textContent = `${motor[2]}-${motor[0]}`;
                            choose_motor_output.appendChild(newOptionDiv);
                        }
                    })
                }
            },
            error: function(xhr, status, error) { // 请求失败时的回调函数
            console.error(error); // 在控制台打印错误信息
            }
        })
    }

    function update_action_form(form_data,form_type){
        let action_select_group=document.getElementById(`${form_type}_action_select_group`)
        const action_tb=document.getElementById(`${form_type}_action_tb`)
        while (action_tb.firstChild) {
            action_tb.removeChild(action_tb.firstChild);
        }
        while (action_select_group.firstChild) {
            action_select_group.removeChild(action_select_group.firstChild);
        }
        action_select_group.insertAdjacentHTML('beforeend',`<option value="0" selected >请选择选择动作组</option>`)
        form_data.forEach(function(action,index){
            $(`#${form_type}_action_tb`).append(`<tr id="action-from-${action.motor_id}"><td>${index}</td><td>${action.location}</td><td>${action.name_index}</td><td>${action.position}</td><td>${action.speed}</td></tr>`)
        })
    }

    function update_control_id(){
        $.ajax({
            type: "get",
            url: "/control/get_max_control_id",
            success: function (response) {
                if(response.maxid){
                    $(`#control_id`).val(response.maxid)
                }
            }
        });
    }

    $(`#add_motor`).click(function() {
        if($(`#protocol`).val()===0){
            alert("协议不能为空")
            loading_off("modify","motor")
            return 
        }
        loading_on("add","motor")
        var data={
            "motor_id":$(`#motor_id`).val(),
            "can_id":$(`#can_id`).val(),
            "name":$(`#motor_name`).val(),
            "protocol":parseInt($(`#protocol`).val()),
            "current_position":$(`#motor_current_position`).val(),
            "max_position":$(`#motor_max_position`).val(),
            "min_position":$(`#motor_min_position`).val(),
            "default_position":$(`#motor_default_position`).val()
        }
        $.ajax({
            type:"post",
            url:"/motor/add",
            data:data,
            success: function(response) { // 请求成功时的回调函数
                alert(response.message)
                if(response.message==="success"){
                    refresh_motor_page()
                }
                loading_off("add","motor")
            },
            error: function(xhr, status, error) { // 请求失败时的回调函数
                console.error(error); // 在控制台打印错误信息
                loading_off("add","motor")
            }
        })
    });

    $(`#add_controller`).click(function(){
        loading_on("add","controller")
        const motor_selects=$(`#choose_motor_output .motor_choose_output_div`)
        const control_type=$(`#control_type`).val()
        const data=get_controller_form(motor_selects,control_type,'')
        if(data===false){
            loading_off("add","controller")
            return
        }
        $.ajax({
            type:"post",
            url:"/controller/add",
            data:data,
            success: function(response) { // 请求成功时的回调函数
                alert(response.message)
                if(response.message==="success"){
                    refresh_control_config_page()
                }
                loading_off("add","controller")
            },
            error: function(xhr, status, error) { // 请求失败时的回调函数
                console.error(error); // 在控制台打印错误信息
                loading_off("add","controller")
            }
        })
    })

    $(`#openModalBtn2`).click(function (){ 
        const motor_select = document.getElementById('motor_select');
        const choose_motor_output = document.getElementById('choose_motor_output');
        update_controller_form(motor_select,choose_motor_output,'',[],"")
        update_control_id()
    })

    $(`#modify_motor`).click(function (){
        loading_on("modify","motor")
        if($(`#modify_protocol`).val()==="0"){
            alert("协议不能为空")
            loading_off("modify","motor")
            return 
        }
        $.ajax({
            type:"put",
            url:"/motor/modify",
            data:{
                "motor_id":parseInt($(`#modify_motor`).attr("data")),
                "can_id":$(`#modify_can_id`).val(),
                "name":$(`#modify_motor_name`).val(),
                "protocol":parseInt($(`#modify_protocol`).val()),
                "current_position":$(`#modify_motor_current_position`).val(),
                "max_position":$(`#modify_motor_max_position`).val(),
                "min_position":$(`#modify_motor_min_position`).val(),
                "default_position":$(`#modify_motor_default_position`).val(),
                "boards_motor":parseInt($(`#modify_motor_id`).val())
            },
            success: function(response) { // 请求成功时的回调函数
                if(response.message==="success"){
                    refresh_motor_page()
                    alert("success")
                }
                else{
                    alert(response.message)
                }
                loading_off("modify","motor")
            },
            error: function(xhr, status, error) { // 请求失败时的回调函数
                console.error(error); // 在控制台打印错误信息
                loading_off("modify","motor")
            }
        })
    })

    $(`#modify_controller`).click(function(){
        loading_on("modify","controller")
        const motor_selects=$(`#modify_choose_motor_output .motor_choose_output_div`)
        const control_type=$(`#modify_control_type`).val()

        let data=get_controller_form(motor_selects,control_type,'modify_')
        if(data===false){
            loading_off("modify","controller")
            return
        }
        number=$(`#modify_target`).text().match(/\d+/)
        data["control_id"]=String(number)
        
        $.ajax({
            type:"put",
            url:"/controller/modify",
            data:data,
            success: function(response) { // 请求成功时的回调函数
                alert(response.message)
                if(response.message==="success"){
                    refresh_control_config_page()
                }
                loading_off("modify","controller")
            },
            error: function(xhr, status, error) { // 请求失败时的回调函数
                console.error(error); // 在控制台打印错误信息
                loading_off("modify","controller")
            }
        })
    })

    function delete_motor(motorId){
        $.ajax({
            type:"delete",
            url:"/motor/delete",
            data:{"motor_id":motorId},
            success: function(response) { // 请求成功时的回调函数
                if(response.message==="success"){
                    refresh_motor_page()
                }
            },
            error: function(xhr, status, error) { // 请求失败时的回调函数
            console.error(error); // 在控制台打印错误信息
            }
        })
    }

    function delete_control(controlId,motor_id){
        $.ajax({
            type:"delete",
            url:"/controller/delete",
            data:{"control_id":parseInt(controlId),"motor_id":parseInt(motor_id)},
            success: function(response) { // 请求成功时的回调函数
                if(response.message==="success"){
                    refresh_control_config_page()
                    group_detail_off()
                }
            },
            error: function(xhr, status, error) { // 请求失败时的回调函数
                console.error(error); // 在控制台打印错误信息
            }
        })
    }

    function refresh_motor_page(){
        while (motor_table.firstChild) {
            motor_table.removeChild(motor_table.firstChild);
        }
    }

    function refresh_control_config_page(){
        while (control_table.firstChild) {
            control_table.removeChild(control_table.firstChild);
        }
        $.ajax({
            type:"get",
            url:"/controller/get_all",
            data:"",
            success: function(response) { // 请求成功时的回调函数
                response.controllers.forEach((element, index) => {
                    let depend=''
                    if (element[9]!==null){
                        depend=element[9]
                    }
                    else{
                        depend="点击添加依赖"
                    }
                    let mode=''
                    if (element[11]===1){
                        mode="位置/速度模式"
                    }
                    if (element[11]===2){
                        mode="力矩/位置模式"
                    }
                    control_table.insertAdjacentHTML('beforeend', 
                        `<tr id="control-config-${element[0]}-${element[8]}">
                            <td  style="width: 2%;">${element[0]}</td>
                            <td>${element[1]}</td>
                            <td>${element[2]}</td>
                            <td>${element[3]}</td>
                            <td mode_data="${element[11]}">${mode}</td>
                            <td>${element[4]}</td>
                            <td>${element[5]}</td>
                            <td>${element[6]}</td>
                            <td>${element[7]}</td>
                            <td>${element[12]}</td>
                            <td>${element[10]}</td>
                            <td style="width: 15%;">
                                <a href="#" id="depends-a-${element[0]}-${element[8]}">${depend}</a>
                                <input type="file" name="depends-upload-${element[0]}-${element[8]}" id="depends-upload-${element[0]}-${element[8]}" style="display: none;" motor_id="${element[8]}" />
                            </td>
                            <td style="padding:0;width:13%">
                                <button class="table_operation_btn" style="height:35px" data-control-modify-id="${element[0]}">编辑</button>
                                <button class="table_operation_btn_waring" style="height:35px" data-control-delete-id="${element[0]}" motor_id="${element[8]}">删除</button>
                            </td>
                        </tr>`);
                });
                },
            error: function(xhr, status, error) { // 请求失败时的回调函数
            console.error(error); // 在控制台打印错误信息
            }
        })
    }

    function refresh_control_center_page(){
        while (control_page.firstChild) {
            control_page.removeChild(control_page.firstChild);
        }
        alert(1)
        $.ajax({
            type:"get",
            url:"/control/get_all",
            data:{"robot_type":$(`#robot_type_select`).val()},
            success: function(response) { // 请求成功时的回调函数
                    response.topic.forEach((data,index)=>{
                        let control_bar=""
                        $(`#control_center`).append(
                            `<div class="contorl-card" id="part_${data}">
                                <div class="card_title" style="display:flex;flex-direction:row;justify-content: space-between;">
                                    <div>
                                        <label for="location_all_select_${data}">${response.part_names[index]}</label><input type="checkbox" id="location_all_select_${data}" class="location_all_select"/>
                                    </div>
                                    <div style="font-size: initial;display: flex;gap: 60px;padding-top: 30px;">
                                        <span>位置</span>
                                        <span>速度</span>
                                        <span>电流</span>
                                    </div>
                                </div>
                                <hr style="margin-top:0">
                            </div>`
                        )
                    });
                    response.motors.forEach((data,index)=>{
                        control_bar=`
                        <div class="form-group card_body">
                            <a href="#" class="spinner-border text-success controller_running" role="status" id="control_bar_loading_${data[0]}" style="display: none;width:1.5rem;height:1.5rem;" data=${data[0]}>
                                <span class="sr-only">Loading...</span>
                            </a>
                            <input type="checkbox" style="width:3%" id="control-checkbox-${data[2]}"  canid="${data[4]}" module="${data[3]}" name_index="${data[2]}">
                            <label for="control-checkbox-${data[2]}">${data[1]}(${data[5]}):</label>
                            <input type="range" class="form-control-range" id="control-${data[2]}" min="${data[7]}" max="${data[6]}" value="${data[5]}" step="0.001" canid="${data[4]}" module="${data[3]}" name_index="${data[2]}" title=""/>
                            <input type="number" class="control-position-input" style="width:10%" value="${data[5]}" id="control-position-${data[2]}" autocomplete="off" step="0.01"  canid="${data[4]}"  module="${data[3]}" name_index="${data[2]}"
                                oninput="if(value>${data[6]})value=${data[6]};if(value<${data[7]}) value=${data[7]}" title=""/>
                            <input type="number" style="width:10%" value="${default_speed}" id="control-speed-${data[2]}" min="0" autocomplete="off" step="0.1"/>
                            <input type="number" style="width:10%" value="${default_current}" id="control-current-${data[2]}" min="0" autocomplete="off" step="0.1"/>
                        </div>`
                        $(`#part_${data[3]}`).append(control_bar)
                    });
                },
            error: function(xhr, status, error) { // 请求失败时的回调函数
            console.error(error); // 在控制台打印错误信息
            }
        })
    };

    $(`#robot_type_select`).change(function(){
        refresh_control_center_page()
    })

    function refresh_action_group(){
        const action_groups_cards_box=document.getElementById("action_groups_cards_box")
        while (action_groups_cards_box.firstChild) {
            action_groups_cards_box.removeChild(action_groups_cards_box.firstChild);
        }
        $.ajax({
            type:"get",
            url:"/action_group/get_all",
            data:"",
            success:function(response){
                response.groups.forEach((data,index)=>{
                    action_groups_cards_box.insertAdjacentHTML('beforeend',`
                        <div class="card" id="action-card-${data[0]}">
                            <div class="card-body">
                                <h5 class="card-title">${data[1]}</h5>
                                <p class="card-text">${data[2]}</p>
                                <hr style="margin-bottom:5px;margin-top:5px">
                                <p class="card-text">${data[3]}</p>
                                <a href="#" class="btn btn-primary group_card_streach" data="${data[0]}" style="position: absolute;bottom: 4%;left: 40%;z-index:1">查看</a>
                                <div class="right_card_body_div">
                                    <a href="#" class="btn btn-primary edit btn-sm" data="${data[0]}">编辑</a>
                                    <a href="#" class="btn btn-warning btn-sm" data="${data[0]}">移动</a>
                                    <a href="#" class="btn btn-danger btn-sm" data="${data[0]}">删除</a>
                                
                                </div>
                                <div class="action_tree_box" id="action_tree_box-${data[0]}">

                                </div>
                            </div>
                        </div>
                        `)
                })
            },
            error:function(e){
                alert(e)
            }
        })
    }

    function loading_on(type1,type2){
        $(`#${type1}_${type2}`).css("display","none")
        $(`#${type1}_${type2}_wait`).css("display","block")
    }

    function loading_off(type1,type2){
        $(`#${type1}_${type2}`).css("display","block")
        $(`#${type1}_${type2}_wait`).css("display","none")
    }

    $(`#motor-config-page-btn`).click(function(){
        refresh_motor_page()
    })

    $(`#control-config-page-btn`).click(function(){
        refresh_control_config_page()
    })

    $(`#control-center-page-btn`).click(function(){
        refresh_control_center_page()
    })

    $(`#action-group-page-btn`).click(function(){
        action_group_details.classList.remove('active')
        refresh_action_group()
    })

    function get_control_form(){
        const control_id_checkbox=$('input[type="checkbox"][id^="control-checkbox-"]').filter(':checked');
        let data=[]
        control_id_checkbox.each(function() {
            let controller={}
            const control_id=$(this).attr('id').split("-")[2]
            controller["id"]=control_id
            controller["position"]=parseFloat($(`#control-position-${control_id}`).val())
            controller["speed"]=parseFloat($(`#control-speed-${control_id}`).val())
            controller["type"]=parseInt($(this).attr("control_mode"))
            data.push(controller)
        });
        return data
    }
    
    function standar_get_control_form() {
        const control_id_checkbox = $('input[type="checkbox"][id^="control-checkbox-"]').filter(':checked');
        let commands = [];
        let hasError = false;
        let errorMessage = "";
        
        control_id_checkbox.each(function() {
            if (hasError) return false; // 如果已经有错误，停止继续处理
            
            let data = {};
            const motor_id = $(this).attr('id').split("-")[2];
            const part = $(this).attr('module');
            
            // 获取原始输入值
            const valueStr = $(`#control-position-${motor_id}`).val();
            const speedStr = $(`#control-speed-${motor_id}`).val();
            const currentStr = $(`#control-current-${motor_id}`).val();
            
            // 验证 value
            if (!isValidNumber(valueStr)) {
                hasError = true;
                errorMessage = `模块 ${part} (ID: ${motor_id}) 的位置值 "${valueStr}" 不是有效的数字`;
                return false;
            }
            
            // 验证 speed
            if (!isValidNumber(speedStr)) {
                hasError = true;
                errorMessage = `模块 ${part} (ID: ${motor_id}) 的速度值 "${speedStr}" 不是有效的数字`;
                return false;
            }
            
            // 验证 speed 不能为0或负数
            const speed = parseFloat(speedStr);
            if (speed <= 0) {
                hasError = true;
                errorMessage = `模块 ${part} (ID: ${motor_id}) 的速度值必须大于0`;
                return false;
            }

            const current = parseFloat(currentStr);
            if (current <= 0) {
                hasError = true;
                errorMessage = `模块 ${part} (ID: ${motor_id}) 的电流值必须大于0`;
                return false;
            }
            
            // 如果所有验证都通过，则添加到命令列表
            data["name_index"] = parseInt($(this).attr("name_index"));
            data["value"] = parseFloat(valueStr);
            data["speed"] = speed;
            data["part"] = part;
            data["current"]=current
            
            commands.push(data);
        });
        
        // 如果有错误，显示弹窗并返回空数组
        if (hasError) {
            showErrorAlert(errorMessage);
            return [];
        }
        
        return commands;
    }

    // 验证是否为有效数字的函数
    function isValidNumber(str) {
        if (str === "" || str === null || str === undefined) {
            return false;
        }
        
        // 检查是否为数字（包括小数和负数）
        const num = parseFloat(str);
        return !isNaN(num) && isFinite(str);
    }

    // 显示错误弹窗的函数
    function showErrorAlert(message) {
        // 使用浏览器原生alert
        alert("输入错误: " + message);
        
        // 或者使用自定义模态框（如果你有UI框架）
        // 示例：使用Bootstrap模态框
        /*
        $('#errorModal .modal-body').text(message);
        $('#errorModal').modal('show');
        */
        
        // 或者使用SweetAlert2（如果已引入）
        /*
        Swal.fire({
            icon: 'error',
            title: '输入错误',
            text: message,
            confirmButtonText: '确定'
        });
        */
    }

    $("#motor_alignment").click(function (e) { 
        e.preventDefault();
        const control_id_checkbox=$('input[type="checkbox"][id^="control-checkbox-"]').filter(':checked');
        if (control_id_checkbox.length==0){
            alert("请选择控制对象")
            return 
        }
        const ids=[]
        control_id_checkbox.each(function(){
            ids.push(parseInt($(this).attr("id").split("-")[2]))
        })
        console.log(ids)
        $.ajax({
            type: "post",
            url: "/control/align",
            contentType: "application/x-www-form-urlencoded; charset=UTF-8",
            traditional: true, // 重要：用于正确序列化数组参数
            data: {"control_ids":ids},
            success: function (response) {
                console.log(response.message)
            },
            error: function(xhr, status, error) { // 请求失败时的回调函数
                console.log(error); // 在控制台打印错误信息
            }
        });
    });

    $(`#run`).click(function(){
        const control_id_checkbox=$('input[type="checkbox"][id^="control-checkbox-"]').filter(':checked');
        if (control_id_checkbox.length==0){
            alert("请选择控制对象")
            return 
        }
        const commands=standar_get_control_form()
        if(commands==false){
            return
        }
        $.ajax({
            type:"post",
            url:"/control/run",
            contentType: "application/json",
            data: JSON.stringify(commands), // 序列化处理后的数据
            dataType: "json",
            success: function(response) { // 请求成功时的回调函数
                // refresh_control_center_page()
                if(response.message!=="success"){
                    alert(response.message)
                }
                console.log(response.message)
            },
            error: function(xhr, status, error) { // 请求失败时的回调函数
            console.log(error); // 在控制台打印错误信息
            }
        }) 
    })

    $(`#control_init`).click(function(){
        const control_id_checkbox=$('input[type="checkbox"][id^="control-checkbox-"]').filter(':checked');
        if (control_id_checkbox.length==0){
            alert("请选择控制对象")
            return 
        }
        $.ajax({
            type:"post",
            url:"/control/init",
            contentType: "application/json",
            data: JSON.stringify({"data": get_control_form()}),
            success: function(response) { // 请求成功时的回调函数
                    console.log(response.message)
                },
            error: function(xhr, status, error) { // 请求失败时的回调函数
            console.error(error); // 在控制台打印错误信息
            }
        }) 
    })

    $(`#open_add_action_modal`).click(function(){
        add_actions_window_open()
    })

    $(`#open_action_group_modal`).click(function(){
        add_actions_group_window_open()
    })

    $('#add_action_select_group').change(function() {
        var selectedValue = $(this).val();
        $.ajax({
            url: "/action/get_all", // 替换为你的服务器端点URL
            type: 'GET',
            data: { "group_id": selectedValue },
            success: function(response) {
                const actions = response.action;
                // 初始化最大速度和对应的子列表
                let maxSeq = 1
                // 遍历 action 列表，找到第5位最大的子列表
                actions.forEach(action => {
                    const seq = action[3]; // 第5位是速度
                    if (seq >= maxSeq) {
                        maxSeq = seq+1;
                        const maxSpeedAction = action;
                    }
                });
                console.log(maxSeq)
                $(`#add_action_seq`).val(maxSeq)
            },
            error: function(xhr, status, error) {
                console.error('AJAX Error:', status, error);
                // 在这里处理错误情况
            }
        });
    });

    $(`#add_action`).click(function(){
        loading_on("add","action_group")
        let form={}
        form["action_name"]=$(`#add_action_name`).val() 
        form["group_id"]=parseInt($(`#add_action_select_group`).val())
        form["description"]=$(`#add_action_description`).val()
        form["seq"]=parseInt($(`#add_action_seq`).val())
        form["command"]=standar_get_control_form()
        console.log(form)
        $.ajax({
            type:"post",
            url:"/action/add",
            contentType: "application/json",
            data: JSON.stringify(form),
            success: function(response) { // 请求成功时的回调函数
                    if (response.message=="success"){
                        alert("success")
                    }
                    else{
                        alert(response.message)
                    }
                    setTimeout(function() {
                        loading_off("add","action_group")
                    }, 200); // 2000毫秒 = 2秒
                },
            error: function(xhr, status, error) { // 请求失败时的回调函数
                console.error(error); // 在控制台打印错误信息
                loading_off("add","action_group")
            }
        })
        
    })
    
    $(`#modify_action`).click(function(){
        let form={}
        form["action_name"]=$(`#modify_action_name`).val() 
        form["group_id"]=parseInt($(`#action-group-name`).attr("data"))
        form["seq"]=parseInt($(`#modify_action_seq`).val())
        form["action_id"]=parseInt($(`#modify-action-modal`).attr("data"))
        let data=[]
        $(`#modify_control_bar input[id^="modify_control-speed-"]`).each(function() {
            console.log($(this))
            const motor_id=$(this).attr('id').split("-")[2]
            let controller={}
            controller["name_index"]=$(this).attr("name_index")
            controller["value"]=parseFloat($(`#modify_control-position-${motor_id}`).val())
            controller["speed"]=parseFloat($(this).val())
            data.push(controller)
            // refresh_action_group()
        });
        form["command"]=data
        console.log(form)
        $.ajax({
            type:"put",
            url:"/action/modify",
            contentType: "application/json",
            data: JSON.stringify(form),
            success: function(response) { // 请求成功时的回调函数
                    if (response.message=="success"){
                        group_detail_on(form["group_id"])
                        alert("success")
                    }
                    else{
                        alert(response.message)
                    }
                    setTimeout(function() {
                        loading_off("modify","action")
                    }, 200); // 2000毫秒 = 2秒
                },
            error: function(xhr, status, error) { // 请求失败时的回调函数
                console.error(error); // 在控制台打印错误信息
                loading_off("modify","action")
            }
        })
    })

    $(`#stop_action_group`).click(function(){
        const group_id=$(`#action-group-name`).attr("data")
        $.ajax({
            type: "post",
            url: "/action_group/stop",
            success: function (response) {
                console.log("shit")
            }
        });
    })

    $(`#add_action_group`).click(function(){
        const formdata=get_add_action_group_form('add')
        $.ajax({
            type:"post",
            url:"/action_group/add",
            data:formdata,
            success:function(response){
                if(response.message){
                    alert("success")
                    $('#add-action-group-modal').css('display', 'none');
                    add_actions_window_open()
                }
            },
            error:function(e){
                alert(e)
            }
        })
    })

    function let_other_card_die(container,card){
        const allCards = container.querySelectorAll(':has(.card-body)');
        allCards.forEach(otherCard => {
            if (otherCard !== card) {
                otherCard.style.display="none"
            }
        });
    }

    function group_detail_on(group_id){
        action_group_details.classList.add('active')
        document.getElementById("action_group_details_tb").innerHTML=""
        $(`#action_tree_box-${group_id}`).empty();
        $(`#action-group-name`).empty();
        $.ajax({
            type:"get",
            url:"/action/get_all",
            data:{"group_id":parseInt(group_id)},
            success:function(response){
                if (response){
                    $(`#action-group-name`).replaceWith(`<h1 id="action-group-name" data="${response.group[0]}">${response.group[1]}</h1>`);
                    response.action.forEach((data,index)=>{
                        const command = data[4].replace(/"/g, "'")
                        const seqUpButton = index==0 ? `<button class="button_seq_disabled" data="${data[0]}" way="up" disabled>升</button>` : `<button class="button_seq"  data="${data[0]}" way="up">升</button>`;
                        const seqDownButton = index==response.action.length-1 ? `<button class="button_seq_disabled" data="${data[0]}" way="down" disabled>降</button>` : `<button class="button_seq" data="${data[0]}" way="down">降</button>`;
                        document.getElementById(`action_group_details_tb`).insertAdjacentHTML("beforeend",`
                            <tr id="action_${data[0]}">
                                <td>${1+index}</td>
                                <td>${data[2]}</td>
                                <td>
                                    <a href="javascript:void(0)" title="${command}" class="action_command">${data[4]}</a>    
                                </td>
                                <td class="action_opertion_father">
                                    ${seqUpButton}
                                    ${seqDownButton}
                                    <button type="button" class="table_operation_btn" data="${data[0]}">编辑</button>
                                    <button type="button" class="table_operation_btn_waring"  data="${data[0]}">删除</button>
                                </td>
                            </tr>`)
                        $(`#action_tree_box-${response.group[0]}`).append(
                            `
                            <div class="tree" data="${data[0]}">
                                <div class="circle">${index+1}</div>
                                <div class="action_name_statu">${data[2]}</div>
                                <div class="action_gap_time_statu"><input type="number" step="0.1" value="3" title="间隔" id="gap_time_${response.group[0]}_${data[0]}">s</div>
                            </div>
                            `)
                    })
                }
            },
            error: function(xhr, status, error) {
                console.error("An error occurred: " + error);
            }
        })
    }
    function group_detail_off(){
        action_group_details.classList.remove('active')
    }

    document.querySelectorAll('#action_groups_cards_box').forEach(container => {
        container.addEventListener('click', (event) => {
            if (event.target && event.target.classList.contains('group_card_streach')) {
                const card=event.target.closest(':has(.card-body)')
                if(card.style.height==="100%"){
                    card.style.height='28.5%';
                    action_group_details.classList.remove('active')
                    setTimeout(() => {
                        refresh_action_group()
                    }, card_strech_time); 
                }
                else{
                    group_detail_on(event.target.getAttribute("data"))
                    card.style.height='100%';
                    let_other_card_die(container,card)
                }
            }
        });
    });

    $(`#modify_action_group`).click(function(){
        const formdata=get_add_action_group_form('modify')
        formdata.group_id=$(`#modify_action_group`).attr("data")
        console.log(formdata)
        $.ajax({
            type:"put",
            url:"/action_group/modify",
            data:formdata,
            success:function(response){
                alert(response.message)
                if(response.message){
                    $('#modify-action-group-modal').css('display', 'none');
                    refresh_action_group()
                    group_detail_off()
                }
            },
            error:function(e){
                alert(e)
            }
        })
    })

    function action_run(command){
        console.log(command)
        $.ajax({
            url: "/action/run",
            method: "post",
            contentType: "application/json",
            data: JSON.stringify(command), // 序列化处理后的数据
            dataType: "json",
            success: function(response) {
            //   alert("发送成功：" + JSON.stringify(response));
            },
            error: function(xhr) {
              const errorMsg = "错误: " + xhr.status + " - " + xhr.responseText;
              console.error(errorMsg);
            //   alert(errorMsg);
            }
          });
    }

    function processCircles(gap_time,commands,index = 0) {
        const circles = document.querySelectorAll('.circle');
        const now = new Date();
        console.log(`Start: ${now.toLocaleTimeString()}`);
        if (index < circles.length) {
            circles[index].classList.add('active'); // 添加 'active' 类
            $(`#action_group_details_tb tr`).eq(index).addClass('active')
            $.ajax({
                type: "post",
                url: "/action/run",
                contentType: "application/json",
                data: JSON.stringify(commands[index]), // 序列化处理后的数据
                dataType: "json",
                success: function (response) {
                    // 转完后的间隔时间
                    console.log(`End: ${now.toLocaleTimeString}`);
                    setTimeout(() => {
                        circles[index].classList.remove('active'); // 1秒后移除 'active' 类
                        $(`#action_group_details_tb tr`).eq(index).removeClass('active');
                        processCircles(gap_time,commands,index + 1); // 处理下一个元素
                    }, gap_time[index]*1000);
                }
            });
        }
    }

    function action_group_run(){
        let commands=[]
        let gap_time=[]
        const group_id=$(`#action-group-name`).attr(`data`)
        $(`#action_tree_box-${group_id} .tree`).each(function(){
            const action_id=$(this).attr(`data`)
            gap_time.push(parseFloat($(`#gap_time_${group_id}_${action_id}`).val()))
        })
        $(`#action_group_details_tb tr td a`).each(function(){
            let jsonStr = $(this).attr("title");

            // 进行格式修正
            jsonStr = jsonStr
                .replace(/'/g, '"')    // 替换所有单引号为双引号
                .replace(/None/g, 'null'); // 替换Python的None为JSON的null
            
            // 现在可以安全解析
            let cmdData = JSON.parse(jsonStr);
            console.log(cmdData)            
            commands.push(cmdData)
        })
        processCircles(gap_time,commands)
    }

    $(`#run_action_group`).click(function(){
        action_group_run()
    })

    function moveInnerCircle(x, y,z) {
        const innerCircle = document.getElementById('innerCircle');
        const circle_direction=document.getElementById('inner-circle-direction')
        let offsetX = -50;
        let offsetY = -50;
        offsetX += 0.2777*x;
        offsetY += 0.2777*y;
        console.log(`偏移(${offsetX}%,${offsetY}%)`)
        innerCircle.style.transform = "translate(" + offsetX + "%," + offsetY + "%)";
        circle_direction.style.transform=`rotate(-${z}deg)`
        
    }

    $(`#skill-page-btn`).click(function(){
        imu_socket = new WebSocket('ws://192.168.31.128:8765'); // Replace with your WebSocket server URL
        imu_socket.onopen = function(event) {
            console.log('WebSocket is open now.');
            $(`#innerCircle`).css("background-color","rgb(63, 197, 146)")
        };

        imu_socket.onmessage = function(event) {
        data=JSON.parse(event.data)
        console.log(data)
        moveInnerCircle(data[0],data[1],data[2])
        };

        imu_socket.onclose = function(event) {
        console.log('WebSocket is closed now.');
        $(`#innerCircle`).css("background-color","rgb(199, 199, 199)")
        };

        imu_socket.onerror = function(error) {
            console.error('WebSocket error:', error);
            $(`#innerCircle`).css("background-color","rgb(199, 199, 199)")
        };
    })

    $(`#menu_options li a`).click(function(){
        if($(this).attr("id")!=="skill-page-btn"){
            if(imu_socket){
                imu_socket.close()
                console.log("imu_socket close test")
            }
        }
        console.log(`redirect to ${$(this).attr("id")}`)
    })

    $(`#run_action_group_smooth`).click(function(){
        const group_id=$(`#action-group-name`).attr('data')
        let gap_time=[]
        let action_ids=[]
        const circulate=($(`#action_group_circulate`).is(":checked") === true) ? false : true
        $(`#action_tree_box-${group_id} .tree`).each(function(){
            const action_id=$(this).attr(`data`)
            action_ids.push(action_id)
            gap_time.push(parseFloat($(`#gap_time_${group_id}_${action_id}`).val()))
        })
        var requestData = {
            group_id: parseInt(group_id),
            gap_time: gap_time,
            cycle:circulate
        };
        $.ajax({
            type: "post",
            url: "/action_group/run",
            contentType: "application/json",
            data: JSON.stringify(requestData),
            success: function (response) {
                if(response){
                    alert(response.message)
                }
            }
        });
    })

    $(`#debug_switch`).change(function(){
        const statu=$(this).is(':checked');
        if(statu){
            $.ajax({
                type: "post",
                url: "/control/debug_on",
                success: function (response) {
                    alert(response.message)
                }
            });
        }
        else{
            $.ajax({
                type: "post",
                url: "/control/debug_off",
                success: function (response) {
                    alert(response.message)
                }
            });
        }
    })

$(`#reset_current`).click(function(){
    const check_boxes = $(`input[type=checkbox][id^="control-checkbox-"]`).filter(':checked');
    let nonArmModules = [];
    let form = {"control_ids": [], "parts": []}
    
    // 首先检查所有选中的复选框
    for (let index = 0; index < check_boxes.length; index++) {
        const checkbox = check_boxes[index];
        const module = $(checkbox).attr('module');
        const control_id = checkbox.id.split("-")[2]; // control_id 是一个字符串，例如 "123"
        
        form["control_ids"].push(parseInt(control_id)); // 推送整数 123
        form["parts"].push(module);                 // <--- 修改这里：直接推送字符串 "123"
    }
    
    $.ajax({
        type: "post",
        url: "/control/reset_current",
        contentType: "application/json",
        data: JSON.stringify(form),
        success: function (response) {
            alert(response.message)
        },
        // 强烈建议添加错误处理，这样可以看到服务器返回的详细错误信息
        error: function(jqXHR, textStatus, errorThrown) {
            console.error("请求失败:", jqXHR.responseText);
            alert("请求失败，请检查控制台获取详细信息。\n" + jqXHR.responseText);
        }
    })
})

    function refresh_rectify_page(){
        const imu_config_table=document.getElementById(`imu_config`)
        while (imu_config_table.firstChild){
            imu_config_table.removeChild(imu_config_table.firstChild)
        }
        $.ajax({
            type: "get",
            url: "/balance/get",
            success: function (response) {
                if(response.message){
                    response.message.imus.forEach(function(data3,index){
                        if(data3[6]===null){
                            data3[6]="未设置"
                        }
                        if(data3[7]===null){
                            data3[7]="未设置"
                        }
                        $(`#imu_config`).append(
                            `<tr id="imu-${data3[0]}">
                                <td>${1+index}</td><td>${data3[1]}</td><td>${data3[2]}</td><td>${data3[3]}</td><td>${data3[4]}</td><td>${data3[5]}</td><td>${data3[6]}</td><td>${data3[7]}</td><td>${data3[9]}</td>
                                <td style="width:15%;">
                                    <button class="table_operation_btn" imu_id="${data3[0]}">编辑</button>
                                    <button class="table_operation_btn_waring" imu_id="${data3[0]}">删除</button>
                                </td>
                            </tr>`
                        )
                    })
                }
            }
        });
    }

    function refresh_imu_form(window_type){
        let form_type=""
        if (window_type==="modify"){
            form_type="modify_"
        }
        const imu_port=document.getElementById(`${form_type}imu_port`)
        const imu_location=document.getElementById(`${form_type}imu_location`)
        const imu_relect_joint=document.getElementById(`${form_type}imu_relect_joint`)
        while (imu_port.children.length>1) {
            imu_port.removeChild(imu_port.children[1]);
        }
        while (imu_location.children.length>1) {
            imu_location.removeChild(imu_location.children[1]);
        }
        while (imu_relect_joint.children.length>1) {
            imu_relect_joint.removeChild(imu_relect_joint.children[1]);
        }
        $.ajax({
            type: "get",
            url: "/balance/get_add_imu_form",
            success: function (response) {
                if(response.message){
                    response.message.port.forEach(function(data1,index){
                        $(`#${form_type}imu_port`).append(`
                                <option value="${data1}">${data1}</option>
                        `)
                    })
                    response.message.location.forEach(function(data2,index){
                        $(`#${form_type}imu_location`).append(`
                                <option value="${data2}">${data2}</option>
                        `)
                    })
                    $(`#${window_type}-imu-modal`).css("display","block")     
                }
            }
        });
    }

    $(`#open_add_imu_window_btn`).click(function(){
        refresh_imu_form("add")
    })

    $(`#rectify-page-btn`).click(function(){
        refresh_rectify_page()
    })

    document.querySelectorAll('#add_imu_form').forEach(container => {
        container.addEventListener('change', (event) => {
            if (event.target && event.target.id==="imu_location") {
                const imu_relect_joint=document.getElementById("imu_relect_joint")
                const imu_config_table=document.getElementById("imu_config")
                while (imu_relect_joint.children.length>1) {
                    imu_relect_joint.removeChild(imu_relect_joint.children[1]);
                }
                const location=$(`#imu_location`).val()
                if(location===""){
                    return
                }
                $.ajax({
                    type: "get",
                    url: "/balance/get_control",
                    data: {"location":String(location)},
                    success: function (response) {
                        if(response.message){
                            console.log(response.message)
                            response.message.forEach(function(data,index){
                                $(`#imu_relect_joint`).append(`
                                    <option value="${data[0]}">${data[1]}</option>  
                                `)
                            })
                        }
                    }
                });
            }
        });
    });

    $(`#imu_add`).click(function(){
        const formData = {
            'imu_port': $('#imu_port').val(),
            'imu_baudrate': $('#imu_baudrate').val(),
            'imu_name': $('#imu_name').val(),
            'imu_standard_value': $('#imu_standard_value').val(),
            'imu_deviation': $('#imu_deviation').val(),
            'imu_axis': $('#imu_axis').val(),
            'imu_location': $('#imu_location').val(),
            'imu_relect_joint': $('#imu_relect_joint').val(),
            'imu_factor':$('#imu_factor').val()
        };
        $.ajax({
            type: 'post',
            url: '/balance/add_imu',
            data: formData,
            success: function(response) {
                if(response.message){
                    refresh_rectify_page()
                }
            },
            error: function(error) {
                console.log('Error:', error);
                // You can add code here to handle an error response from the server
            }
        });
    })
    
    $("[name='rectify']").on("click",function(){
        const bus=$(this).attr("imu_bus")
        const boards_imu=parseInt($(this).attr("boards_imu"))
        $.ajax({
            type: "post",
            url: "/balance/rectify",
            data: {"imu_bus":bus,"boards_imu":boards_imu},
            success: function (response) {
                alert(response.message)
            }
        });
    })

    document.querySelectorAll("#imu_config_box").forEach(container => {
        container.addEventListener('click', (event) => {
            if(event.target && event.target.classList.contains('table_operation_btn_waring')) {
                $.ajax({
                    type: "delete",
                    url: "/balance/imu_delete",
                    data: {"imu_id":parseInt(event.target.getAttribute("imu_id"))},
                    success: function (response) {
                        if(response.message){
                            refresh_rectify_page()
                        }
                    }
                });
            }
            if(event.target && event.target.classList.contains('table_operation_btn')) {
                refresh_imu_form("modify")
                
            }
        })
    })
    
    // 初始化数据存储
    let dataset = {
        x: [],
        y: [],
        z: []
    };

    // 配置参数
    const TIME_WINDOW = 60000; // 60秒时间窗口
    const LINE_COLORS = ['#5470C6', '#91CC75', '#EE6666'];

    const option = {
        title: { text: 'Real-time Gyroscope Data' },
        tooltip: {
            trigger: 'axis',
            formatter: function (params) {
                return params.map(p => {
                    const date = new Date(p.value[0]);
                    return `${p.seriesName}<br/>
                    ${date.toLocaleTimeString()}.${p.value[0] % 1000}<br/>
                    Value: ${p.value[1].toFixed(2)}°/s`
                }).join('<hr/>');
            }
        },
        legend: { data: ['X Axis', 'Y Axis', 'Z Axis'] },
        xAxis: {
            type: 'time',
            axisLabel: {
                formatter: function (value) {
                    const date = new Date(value);
                    return echarts.time.format(date, '{HH}:{mm}:{ss}', false);
                }
            }
        },
        yAxis: {
            name: 'Angular Velocity (°/s)' ,
            min:-180,
            max:180
        },
        series: [
            createSeries('X Axis', LINE_COLORS[0]),
            createSeries('Y Axis', LINE_COLORS[1]),
            createSeries('Z Axis', LINE_COLORS[2])
        ]
    };

    function createSeries(name, color) {
        return {
            name: name,
            type: 'line',
            showSymbol: false,
            lineStyle: { color: color },
            data: []
        };
    }

    // WebSocket连接
    const ws = new WebSocket('ws://localhost:1231');

    ws.onmessage = function (event) {
        const newData = JSON.parse(event.data);
        const now = Date.now();

        // 更新数据集（假设数据格式为[x,y,z]）
        updateDataset('x', now, newData[0]);
        updateDataset('y', now, newData[1]);
        updateDataset('z', now, newData[2]);

        // 动态调整显示范围
        option.xAxis.min = now - TIME_WINDOW;
        option.xAxis.max = now;

        myChart.setOption({
            xAxis: option.xAxis,
            series: [
                { data: dataset.x },
                { data: dataset.y },
                { data: dataset.z }
            ]
        });
    };

    function updateDataset(axis, timestamp, value) {
        // 添加新数据点
        dataset[axis].push([timestamp, value]);
        
        // 清理旧数据（两种方式二选一）
        // 方式1：基于时间窗口
        const cutoff = timestamp - TIME_WINDOW;
        while (dataset[axis].length > 0 && dataset[axis][0][0] < cutoff) {
            dataset[axis].shift();
        }

        // 方式2：基于固定数据长度（注释方式1后启用）
        // if (dataset[axis].length > 500) {
        //     dataset[axis].shift();
        // }
    }

    var chartDom = document.getElementById('IMU_charts');
    if (chartDom){
        var myChart = echarts.init(chartDom);
        myChart.setOption(option);
    }

    document.querySelectorAll('#imu_show').forEach(container => {
        container.addEventListener('click', (event) => {
            if (event.target && event.target.classList.contains('btn-outline-info')) {
                console.log(event.target.getAttribute("data"))
                // 修改图表标题
                option.title.text = 'Reset Chart - Real-time Gyroscope Data';
                
                // 清空数据集
                dataset.x = [];
                dataset.y = [];
                dataset.z = [];
                
                // 重置时间窗口到当前时刻
                const now = Date.now();
                option.xAxis.min = now - TIME_WINDOW;
                option.xAxis.max = now;
                
                // 强制应用新配置并清空图表
                myChart.setOption(option, { notMerge: true });
            }
        });
    });

    $(`#knowledge-table tr td .table_operation_btn`).click(function(){
        const self = this; // 保存当前按钮的引用
        const modify_box = document.getElementById("modify-knowledge-modal");
        // 首先更新动作组，并在完成后执行后续操作
        update_action_group_knowledge("modify").done(function() {
            // 现在下拉菜单已填充，获取知识详情并设置值
            $.ajax({
                type: "get",
                url: url + "/knowledge/get",
                data: {"knowledge_id": parseInt($(self).attr("data"))},
                success: function (response) {
                    if (response.message) {
                        $(`#modify_answer_type`).val(response.message.type);
                        $(`#modify_question`).val(response.message.question);
                        $(`#modify_answer`).val(response.message.answer);
                        if (response.message.type == 2 && response.message.group_id) {
                            $(`#modify_bind_action_group`).parent().css("display", "flex");
                            $(`#modify_bind_action_group`).val(response.message.group_id);
                        }
                        else{
                            $(`#modify_bind_action_group`).parent().css("display", "none");
                        }
                        modify_box.style.display = "block";
                        $(`#modify_knowledge_submit`).attr("data", response.message.id);
                    }
                }
            });
        });
    });
    $(`#knowledge-table tr td .table_operation_btn_waring`).click(function(){
        const knowledge_id=parseInt($(this).attr("data"))
        $.ajax({
            type: "delete",
            url: "/knowledge/delete",
            data: {"knowledge_id":knowledge_id},
            success: function (response) {
                if(response.message){
                    $(`#knowledge_id_${knowledge_id}`).remove()
                }
            }
        });
    })


    function get_knowledge_form(type){
        let group_id=parseInt($(`#${type}_bind_action_group`).val())
        if(!group_id){
            group_id=-1
        }
        const form = {
            "answer_type": parseInt($(`#${type}_answer_type`).val()),
            "action_group":group_id,
            "question": String($(`#${type}_question`).val()), // Changed from parseInt to direct value
            "answer": String($(`#${type}_answer`).val())      // Changed from parseInt to direct value
        };
        return form
    }

    $(`#add_knowledge_submit`).click(function(){
        const form= get_knowledge_form("add")
        $.ajax({
            type: "post",
            url: url + "/knowledge/add",
            contentType: "application/json",
            dataType: "json", 
            data: JSON.stringify(form),
            success: function (response) {
                if(response.message){
                    alert(response.message)
                    window.location.reload()
                }
            },
            error: function (xhr, status, error) {
                console.error("Error occurred: " + error);
            }
        });
    });

    $(`#modify_knowledge_submit`).click(function(){
        let form= get_knowledge_form("modify")
        form["knowledge_id"]=parseInt($(this).attr("data"))
        $.ajax({
            type: "put",
            url: url + "/knowledge/modify",
            data: form,
            success: function (response) {
                if(response.message){
                    for (let key in form) {
                        $(`#knowledge_id_${form["knowledge_id"]} td[name="${key}"]`).text(form[key])
                    }
                    const tr=$(`#knowledge_id_${form["knowledge_id"]}`)
                    console.log(tr)
                    tr.addClass('highlight');
                    setTimeout(function() {
                        tr.removeClass('highlight');
                    }, 3000);
                }
            },
            error: function (xhr, status, error) {
                console.error("Error occurred: " + error);
            }
        });
    });

    function update_action_group_knowledge(type){
        const bind_action_group=document.getElementById(`${type}_bind_action_group`)
        while (bind_action_group.children.length>1) {
            bind_action_group.removeChild(bind_action_group.children[1]);
        }
        return $.ajax({
            type: "get",
            url: "/knowledge/get_action_groups",
            success: function (response) {
                response.action_group.forEach(function(data,index){
                    $(`#${type}_bind_action_group`).append(
                        `<option value="${data.id}">${data.name}-${data.id}</option>  `
                    )
                })
            }
        });
    }

    $(`#knowledge-add`).click(function(){
        update_action_group_knowledge("add")
    })


    $(`#action_record`).click(function(){
        $(`#action_record_start`).css("display","none")
        $(`#action_record_end`).css("display","flex")
    })

    $(`#action_record_end`).click(function(){
        $(`#action_record_end`).css("display","none")
        $(`#action_record_start`).css("display","flex")
    })
});
