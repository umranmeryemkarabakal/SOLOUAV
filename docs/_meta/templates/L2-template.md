---
topic: <topic-id>
level: L2
title: "<başlık>"
status: draft
code_refs:
  - <path>:L<xx>-L<yy>
updated: <YYYY-MM-DD>
---

# <Başlık>

> **Kimin için:** Projeye yeni katılan deneyimli mühendis.
> **Ön koşul:** <dil> + <alanın temeli>.
> **Hedef uzunluk:** 2000–3000 kelime.

## Genel bakış

<Bileşenin sorumluluğu ve sınırları. Neyi yapmıyor?>

## Mimari

```mermaid
flowchart TD
    <veri akışı veya durum makinesi>
```

## Arayüz sözleşmesi

| Girdi | Tip | Kaynak | Kısıtlar |
|-------|-----|--------|----------|

| Çıktı | Tip | Tüketici | Garantiler |
|-------|-----|----------|------------|

## Mekanizma

<Adım adım. Sözde kod zorunlu:>

```
<sözde kod — gerçek kod DEĞİL>
```

Gerçek uygulama: `<path>:L<xx>-L<yy>`

## Uç durumlar

> DOCS_SPEC: en az 2 zorunlu.

| Uç durum | Davranış | Kodda |
|----------|----------|-------|

## Performans karakteristiği

<Karmaşıklık, döngü frekansı, bellek, gecikme — hangisi anlamlıysa.
Ölçüm mü analiz mi olduğunu belirt.>

## Ne zaman değiştirmeniz gerekir

<Bu bileşene dokunmayı gerektiren durumlar ve nereden başlanacağı.>

---

**Daha derine →** [L3: <başlık>](./L3-<topic-id>.md)
