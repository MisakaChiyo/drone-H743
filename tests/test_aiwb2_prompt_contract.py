from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_aiwb2_prompt_symbol_is_preserved_for_transparent_entry() -> None:
    app_uart = read("App/Src/app_uart.c")
    app_aiwb2 = read("App/Src/app_aiwb2.c")

    assert 'line[start_index] == \'>\'' not in app_uart
    assert 'APP_AiWB2_ProcessLine(normalized);' in app_uart
    assert 'line[0] == \'>\'' in app_aiwb2
    assert 'aiwb2_is_transparent_prompt(line)' in app_aiwb2


def test_wifi_diag_escapes_transparent_mode_before_at_queries() -> None:
    app_aiwb2 = read("App/Src/app_aiwb2.c")

    assert 'aiwb2_provision_commands[2] = (APP_AiWB2Command){ "AT+WJAP?"' in app_aiwb2
    assert 'aiwb2_provision_commands[6] = (APP_AiWB2Command){ "AT+SOCKETTT"' in app_aiwb2
    assert "aiwb2_state = APP_AIWB2_STATE_ESCAPE_BEFORE;" in app_aiwb2
    assert 'APP_AiWB2_SendRawCommand(commands[i])' not in app_aiwb2
