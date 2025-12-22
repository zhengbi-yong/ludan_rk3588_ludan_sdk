#include "dds/dds.h"
#include "../generated/rt_lowcmd.h"   
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    dds_entity_t participant;
    dds_entity_t topic;
    dds_entity_t reader;
    dds_return_t rc;

    /* 1. 创建 DomainParticipant（Domain ID = 0） */
    participant = dds_create_participant(0, NULL, NULL);
    if (participant < 0) {
        printf("dds_create_participant: %s\n",
               dds_strretcode(-participant));
        return EXIT_FAILURE;
    }

    /* 2. 创建 Topic */
    topic = dds_create_topic(
        participant,
        &xixilowcmd_msg_LowCmd_desc,
        "rt/lowcmd",
        NULL,
        NULL
    );
    if (topic < 0) {
        printf("dds_create_topic: %s\n",
               dds_strretcode(-topic));
        dds_delete(participant);
        return EXIT_FAILURE;
    }

    /* 3. 创建 DataReader */
    reader = dds_create_reader(participant, topic, NULL, NULL);
    if (reader < 0) {
        printf("dds_create_reader: %s\n",
               dds_strretcode(-reader));
        dds_delete(participant);
        return EXIT_FAILURE;
    }

    printf("[Subscriber] Domain=0, Topic=rt/lowcmd\n");

    /* 4. 接收并完整打印 30 个关节 */
    while (1) {
        xixilowcmd_msg_LowCmd msg;
        void *samples[1] = { &msg };
        dds_sample_info_t infos[1];

        rc = dds_take(reader, samples, infos, 1, 1);
        if (rc < 0) {
            printf("dds_take: %s\n", dds_strretcode(-rc));
            break;
        }

        if (rc > 0 && infos[0].valid_data) {
            printf("========== LowCmd received ==========\n");
            printf("mode_pr      = %u\n", msg.mode_pr);
            printf("mode_machine = %u\n", msg.mode_machine);

            for (int i = 0; i < 30; i++) {
                printf(
                    "Joint[%02d]: "
                    "mode=%u  "
                    "q=% .6f  "
                    "dq=% .6f  "
                    "kp=% .6f  "
                    "kd=% .6f  "
                    "tau=% .6f\n",
                    i,
                    msg.motor_cmd[i].mode,
                    msg.motor_cmd[i].q,
                    msg.motor_cmd[i].dq,
                    msg.motor_cmd[i].kp,
                    msg.motor_cmd[i].kd,
                    msg.motor_cmd[i].tau
                );
            }

            printf("crc = 0x%08X\n", msg.crc);
            printf("=====================================\n\n");
        }

        dds_sleepfor(DDS_MSECS(20));
    }

    /* 5. 清理 */
    dds_delete(participant);
    return EXIT_SUCCESS;
}
