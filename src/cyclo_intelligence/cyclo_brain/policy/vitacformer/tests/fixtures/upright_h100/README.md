# Upright H100 contract fixture

Source: https://huggingface.co/Dongkkka/Task000608_Upright_ViTacFormer_H100_B512_Hand_Intern

Revision: `71ac7bf6cce3460192a13386b6268c4849fa09a9`.

`inference_config.json` is unchanged from the source package. SHA256:
`c6a2cce55589a5d21dba736911577f69bfd0f853633d2dbe86864fa242acc6a1`.
This matches its published `SHA256SUMS`.

Tests derive the three loading-related train-config fields from this fixture;
they match the same revision's train_config.json. Synthetic normalization and
checkpoint metadata are created only in temporary test directories.
No trained weights are included or downloaded by the tests.

The source package identifies the model as an incomplete research checkpoint,
not approved for powered deployment. Offline compatibility tests do not change
that status or authorize robot execution.
