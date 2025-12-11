document.addEventListener('DOMContentLoaded', () => {
    const navLinks = document.querySelectorAll('.menu');
    const main_content = document.querySelectorAll('.main-content');
    const motor_modal = document.getElementById("motor-modal");
    const modify_motor_modal=document.getElementById("motor-modify-modal");
    const control_modal = document.getElementById("control-modal");
    const modify_control_modal=document.getElementById("control-modify-modal");
    const select_motor = document.getElementById('motor_select');
    const modify_select_motor = document.getElementById('modify_motor_select');
    const select_control_type = document.getElementById('control_type');
    const modify_select_control_type = document.getElementById('modify_control_type');
    

    navLinks.forEach((link,index) => {
        link.addEventListener('click', () => {
            navLinks.forEach(l => l.classList.remove('active')); // Remove active class from all links
            main_content.forEach(m => m.classList.remove('active'))
            link.classList.add('active'); // Add active class to clicked link
            main_content[index].classList.add("active")
        });
    });

    // Open the modal when the button is clicked
    $("#openModalBtn1").click(function() {
        motor_modal.style.display = "block";
    })
    $("#openModalBtn2").click(function() {
        control_modal.style.display = "block";
        $('.union_control').remove();
        $(`#choose_motor_output`).children('div').remove();
    })


    // Close the modal when the close button is clicked
    // $(`#closeModalBtn1`).click( function() {
    //     motor_modal.style.display = "none";
    // })
    // $(`#closeModalBtn2`).click( function() {
    //     control_modal.style.display = "none";
    //     select_control_type.selectedIndex = 0;
    // })
    // $(`#closeModalBtn3`).click( function() {
    //     modify_motor_modal.style.display = "none";
    // })
    // $(`#closeModalBtn4`).click( function() {
    //     modify_control_modal.style.display = "none";
    // })

    $('.close-btn').click(function() {
        var modalId = $(this).closest('.modal').attr('id');
        $(`#${modalId}`).css('display', 'none');
    });

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

    if (select_control_type){
        select_control_type.addEventListener('change',function(){
            $(`#choose_motor_output`).children('div').remove();
            const control_type=select_control_type.value
            modify_control_form(control_type,"add")
        })
    }

    if(modify_select_control_type){
        modify_select_control_type.addEventListener('change',function(){
            $(`#modify_choose_motor_output`).children('div').remove();
            const control_type=modify_select_control_type.value
            modify_control_form(control_type,"modify")
        })
    }

    function listening_select_motor(control_type,select_motor,outputDiv){
        const selectedOption = select_motor.options[select_motor.selectedIndex];
        if (selectedOption.value !== "0") {
    
            // 检查是否已经存在相同的选项
            let optionExists = false;
            const existingOptions = outputDiv.querySelectorAll('.motor_choose_output_div');
            existingOptions.forEach(optionDiv => {
                if (optionDiv.textContent === `${selectedOption.text}`) {
                    optionExists = true;
                }
            });
            // 如果不存在相同的选项，则添加新的选项
            if (!optionExists) {
                if(control_type==="0"){
                    alert("请选择控制类型")
                    select_motor.selectedIndex = 0;
                    return
                }
                if(control_type==="1"){
                    if(outputDiv.children.length>0){
                        alert("单独控制仅可选择一个电机进行控制")
                        select_motor.selectedIndex = 0;
                        return
                    }
                }
                const newOptionDiv = document.createElement('div');
                newOptionDiv.className = 'motor_choose_output_div';
                newOptionDiv.textContent = `${selectedOption.text}`;
                outputDiv.appendChild(newOptionDiv);
            }
        }
        select_motor.selectedIndex = 0;
    }

    if (select_motor){
        select_motor.addEventListener('change', function() {
            const control_type=$(`#control_type`).val()
            const outputDiv = document.getElementById('choose_motor_output');
            listening_select_motor(control_type,select_motor,outputDiv)
        });
    }

    if (modify_select_motor){
        modify_select_motor.addEventListener("change",function(){
            const control_type=$(`#modify_control_type`).val()
            const outputDiv = document.getElementById('modify_choose_motor_output');
            listening_select_motor(control_type,modify_select_motor,outputDiv)
        })
    }
    
    $(document).on('input', '.modify-form-control-range', function() {
        let rangeValue = $(this).val();
        let textInputId = '#modify_control-position-' + $(this).attr('id').split('-')[1];
        $(textInputId).val(rangeValue);
    });

    $(document).on('input', '.form-control-range', function() {
        let rangeValue = $(this).val();
        let textInputId = '#control-position-' + $(this).attr('id').split('-')[1];
        $(textInputId).val(rangeValue);
    });

    $(document).on('input', '.control-position-input', function() {
        let textInputId = $(this).val();
        let rangeValue = '#control-' + $(this).attr('id').split('-')[2];
        $(rangeValue).val(textInputId);
    });

    $(document).on('input', '.modify-control-position-input', function() {
        let textInputId = $(this).val();
        let rangeValue = '#modify_control-' + $(this).attr('id').split('-')[2];
        $(rangeValue).val(textInputId);
    });

    $(`#imu-table-change`).click(function(){
        const imu_config_box=document.getElementById("imu_config_box")
        if(imu_config_box.style.height==="1px"){
            imu_config_box.style.height="84vh"
        }
        else{
            imu_config_box.style.height="1px"
        }
    })

    $(`#add_group_window_open`).click(function(){
        const add_action_group_modal=document.getElementById("add-action-group-modal")
        add_action_group_modal.style.display="block"
    })

    function initializeScrollSync() {
        var left = document.querySelector(".action_tree_box"); // 使用正确的类选择器
        var right = document.getElementById("action_group_details");

        if (left && right) {
            // 绑定左边的滚动事件
            left.addEventListener('scroll', function() {
                var a = left.scrollTop;
                // 竖向滚动条同步
                right.scrollTop = a;
                // 横向滚动条同步
                right.scrollLeft = left.scrollLeft;
            });
        } else {
            console.error("Elements not found!");
        }
    }

    document.querySelectorAll('#modify_control_bar').forEach(container => {
        container.addEventListener('click', (event) => {
            if (event.target && event.target.classList.contains('table_operation_btn_waring')){
                event.target.closest(`.form-group`).remove()
            }
        });
    });
});